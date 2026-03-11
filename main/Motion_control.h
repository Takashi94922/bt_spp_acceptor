#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include <string.h>

#define TAG "MOTION"
struct PID {
	float Kp, Ki, Kd;
	float prev_error;
	float integral;
	float target = 0; // 目標値（セットポイント）
	float calculatePID(float current, float dt);
};
class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
	dspm::Mat trans;
	//Body座標系でみたときの重心位置[m]
	float x_IMU_src[3] = {41.254E-3, 1.061E-3, 53.987E-3};
	dspm::Mat x_IMU = dspm::Mat(x_IMU_src, 3,1);

	float g_prev_src[3] = {0.0f, 0.0f, 0.0f};
	dspm::Mat g_prev = dspm::Mat(g_prev_src, 3,1);

	float IMU2body_src[9] = {
		0, 0, -1,
		-1, 0, 0,
		0, -1, 0
	};
	dspm::Mat IMU_2_body = dspm::Mat(IMU2body_src, 3, 3);
public:
	Motion_control(){
		float Bsrc[] = {
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0,

			0, -225.0f*3.1415f/100.0f/mass, -225.0f*3.1415f/100.0f/mass, 225.0f*3.1415f/100.0f/mass, 225.0f*3.1415f/100.0f/mass,
			0, 225.0f*3.1415f/100.0f/mass, -225.0f*3.1415f/100.0f/mass, -225.0f*3.1415f/100.0f/mass, 225.0f*3.1415f/100.0f/mass,
			1.69f/100.0f/mass, 0, 0, 0, 0,
		};
		B = dspm::Mat(Bsrc, 6, 5);

		float Qsrc[] = {
			0.0003, 0, 0, 0, 0, 0,
			0, 0.0003, 0, 0, 0, 0,
			0, 0, 0.0003, 0, 0, 0,
			0, 0, 0, 0.0003, 0, 0, 
			0, 0, 0, 0, 0.0003, 0, 		
			0, 0, 0, 0, 0, 0.0003
		};
		Q = dspm::Mat(Qsrc, 6, 6);

		float Rsrc[] = {
			0.5, 0, 0,
			0, 0.5, 0,
			0, 0, 0.5,
		};
		R = dspm::Mat(Rsrc, 3, 3);

		for (int i = 0; i < 5; i++)
		{
			u(i, 0) = 0.0;	
		}

		float KCsrc[] = {
			0,0,0,0,0,0,
			-2.5303691,-6.3249978,4.4290784,-44.534649,-103.852,1.7407752,
			2.5548081,-6.1235034,0.0396076,42.480473,-101.7184,-0.0407247,
			2.5303691,6.3249978,-4.4290784,44.534649,103.852,-1.7407752,
			-2.5548081,6.1235034,-0.0396076,-42.480473,101.7184,0.0407247,
		};
		KC = dspm::Mat(KCsrc, 5, 6);

		//{pitch roll yaw}
		float KPIDsrc[] = {
			0,0,0,
			-1, -1, 0,
			1, -1, 0,
			1, 1, 0,
			-1, 1, 0,	
		};
		KPID = dspm::Mat(KPIDsrc, 5, 3);

		// PID制御用のインスタンス
		//吸い込み力と釣り合わせるためpitchだけちょっと後ろに傾ける
		pitch_pid = {12.0f, 3.0f, 0.1f, 0.0f, 0.0f, 0.05f};
		roll_pid = {10.0f, 2.0f, 0.1f, 0.0f, 0.0f, 0.0f};
		yaw_pid = {.0f, .0f, 0.00f, 0.0f, 0.0f, 0.0f};
	}

	static constexpr float gravity_c = 9.80665;
	static constexpr float deg2rad = 0.0174533;
	static constexpr float rad2deg = 1.0/deg2rad;
	static constexpr float mass = 1.66f;
	static constexpr float dt = 10E-3;
	float gv[3] = {0.0, 0.0, -gravity_c};
	float a0[3] = {0};
	float g0[3] = {0};
	float m0[3] = {0};
	dspm::Mat a_grav = dspm::Mat(3, 1);
	dspm::Mat a = dspm::Mat(3, 1);
	dspm::Mat g = dspm::Mat(3, 1);
	dspm::Mat m = dspm::Mat(3, 1);
	dspm::Mat v = dspm::Mat(3, 1);
	dspm::Mat p = dspm::Mat(3, 1);
	dspm::Mat y = dspm::Mat(3, 1);
	dspm::Mat d_omega = dspm::Mat(4, 4);
	dspm::Mat q = dspm::Mat(4, 1);
	dspm::Mat dq = dspm::Mat(4, 1);
	dspm::Mat u = dspm::Mat(5, 1);
	dspm::Mat a_imu = dspm::Mat(3, 1);
	dspm::Mat g_imu = dspm::Mat(3, 1);
	dspm::Mat m_imu = dspm::Mat(3, 1);
	dspm::Mat gv_imu = dspm::Mat(3, 1);
	dspm::Mat centripetal = dspm::Mat(3, 1);
	dspm::Mat ga = dspm::Mat(3, 1);
	dspm::Mat tangential = dspm::Mat(3, 1);
	dspm::Mat skewM = dspm::Mat(3, 3);
	dspm::Mat Phi = dspm::Mat(6, 6);
	dspm::Mat temp = dspm::Mat(6, 6);
	dspm::Mat F = dspm::Mat(6, 6);
	dspm::Mat H = dspm::Mat(3, 6);
	dspm::Mat K = dspm::Mat(6, 3);
	//単位はrad
	float PRY_value[3] = {0};
	dspm::Mat B, Q, R, KC, KPID;
	float *KCsrc = KC.data;
	dspm::Mat P = dspm::Mat::eye(6);
	dspm::Mat xhat = dspm::Mat(6, 1);
	float gdot[3] = {0};
	float thetadot[3] = {0};
	PID pitch_pid, roll_pid, yaw_pid;
	uint8_t ControlMethod = 0; // 0: None, 1: KC, 2: PID

	void begin(float sampleFreq, i2c_master_bus_handle_t bus_handle);
    void skew(dspm::Mat &v );
	void Sensor2Body();
    void filterUpdate();
    void update();
    void calcU();
	//単位はrad
    void getPRY(float *retbuf);
    void calib();
    void correctInitValue(uint16_t num_loop);
	float calculatePID(PID &pid, float current);
	void rotate(dspm::Mat &out, const dspm::Mat &q, const dspm::Mat &a);
};