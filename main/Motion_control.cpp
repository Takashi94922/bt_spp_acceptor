#include "Motion_control.h"
#include <inttypes.h>
#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include "mat.h"

namespace dspm_utils {

  // 3×1 行列から 3×3 のスキュー対称行列を生成
  dspm::Mat skew( dspm::Mat &v )
  {
    dspm::Mat M(3, 3);

    // v(i,0) は i 行目の要素
    M(0,0) =  0;         M(0,1) = -v(2,0);  M(0,2) =  v(1,0);
    M(1,0) =  v(2,0);    M(1,1) =  0;        M(1,2) = -v(0,0);
    M(2,0) = -v(1,0);    M(2,1) =  v(0,0);  M(2,2) =  0;

    return M;
  }
} // namespace dspm_utils

void Motion_control::begin(float sampleFreq, i2c_master_bus_handle_t bus_handle){
 	if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == 0){
        ESP_LOGE(TAG, "imu initialize faile");
    }
	dt =  1.0f / sampleFreq; //sampleFreqはHzなので、dtは秒
	madgwick.begin(sampleFreq);
}

void Motion_control::Sensor2Body(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz){
	float a_src[3] = {ax, ay, az};
	a = IMU_2_body * dspm::Mat(a_src, 3, 1);
	float g_src[3] = {gx, gy, gz};
	g = IMU_2_body * dspm::Mat(g_src, 3, 1);

	//ジャイロの前回値を使うのでg=より先に計算
	dspm::Mat centripetal = dspm_utils::skew(g) * (dspm_utils::skew(g) * dspm::Mat(x_IMU, 3, 1));

	dspm::Mat ga = ((g - g_prev) * (1.0f /dt));
    dspm::Mat tangential = dspm_utils::skew(ga) * dspm::Mat(x_IMU, 3, 1);
	a = a - centripetal - tangential; //重心の座標系に変換

	float m_src[3] = {mx, my, mz};
	m = IMU_2_body_mag * dspm::Mat(m_src, 3, 1);

	g_prev = g;
}
void Motion_control::filtaUpdate(){
	//ESP_LOGD(TAG, "kalman Start");
	//x = F*x + B*u + Q
	//y = H*x
	
	//x = [ax ay az vx vy vz]';
	
	//y= [ax ay az];
	float ysrc[3] = {a(0, 0), a(1, 0), a(2, 0)};
	dspm::Mat y(ysrc, 3, 1);
	
	//x = *** + Bu
	//Lift = -318.2*alpha cos成分は /1.4142 => 225.0*alpha
 	//u = [Thrust S1 S2  S3 S4]

	//x = Fx + Bu;
	//xhat = F * xhat + B * u;
	xhat = F * xhat;

	//P = F P F' + Q
	P = F*P*F.t() + Q;

	//K = P*H'*(R + H*P*H')^-1
	dspm::Mat K(6, 3);
	K = P*H.t()*(R + H*P*H.t()).inverse();

	//xhat = xhat + K(y-Hx)
	xhat += K*(y-H*xhat);

	//P = (I-KH)P
	P = (dspm::Mat::eye(6) - K*H)*P;
	//ESP_LOGI(TAG, "Kalman Updated");
}
void Motion_control::update(){
	imu.readTemp();
    imu.readAccel();
    imu.readGyro();
    imu.readMag();

	//センサ取り付け角度の変換行列Rinst IMU座標系から重心の座標系へ
	Sensor2Body(imu.calcGyro(imu.gx) * deg2rad, imu.calcGyro(imu.gy) * deg2rad, imu.calcGyro(imu.gz) * deg2rad,
				imu.calcAccel(imu.ax) * gravity_c, imu.calcAccel(imu.ay) * gravity_c, imu.calcAccel(imu.az) * gravity_c,
				imu.calcMag(imu.mx - m0[0]), imu.calcMag(imu.my - m0[1]), imu.calcMag(imu.mz - m0[2]));

	//センサの姿勢を計算
	//azだけ重力加速度込みの値を入れる
	madgwick.update(g(0, 0) *rad2deg, g(1, 0) *rad2deg, g(2, 0) *rad2deg,
					a(0, 0), a(1, 0), a(2, 0),
					m(0, 0), m(1, 0), m(2, 0));
	
	getPRY(PRY_value);

	//姿勢から重力の分力を減算
	float gv[] = {0.0, 0.0, -gravity_c};
	float gv_bsrc[3] = {0.0};

	madgwick.trans(gv_bsrc, gv);

	dspm::Mat gv_b(gv_bsrc, 3, 1);

	//重力加速度を引く
	a = a + gv_b;

	filtaUpdate();

	calcU();
	
	//ESP_LOGI(TAG, "raw%1.2f,%1.2f,%1.2f", a(0, 0), a(1, 0), a(2, 0));
	//ESP_LOGI(TAG, "u%2.1f,%2.1f,%2.1f", u(1, 0), u(2, 0), u(3, 0));
}
void Motion_control::calcU(){
	if (ControlMethod == 0 || ControlMethod == 1) {
		float xsrc[] = {madgwick.getRollRadians(), madgwick.getPitchRadians(), 0, g(0, 0), g(1, 0), g(2, 0)};
		u = -1 * KC * dspm::Mat(xsrc, 6, 1);
	}
	else if (ControlMethod == 2) {
		// PID制御を適用
		float xsrc[3];
		xsrc[0] = calculatePID(pitch_pid, PRY_value[0]);
		xsrc[1] = calculatePID(roll_pid, PRY_value[1]);
		xsrc[2] = calculatePID(yaw_pid, PRY_value[2]);

    	// 制御出力を使用して次の処理を実行
    	u = KPID * dspm::Mat(xsrc, 3, 1);
	}
}

// PID制御計算関数
float Motion_control::calculatePID(PID &pid, float current) {
    float error = pid.target - current; // 誤差
    pid.integral += error * dt;     // 積分項
    float derivative = (error - pid.prev_error) / dt; // 微分項
    pid.prev_error = error;         // 前回の誤差を更新

    // PID制御出力
    return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}

// PRY値を取得する関数単位は度
void Motion_control::getPRY(float* retbuf){
	retbuf[0] = madgwick.getPitchRadians();
	retbuf[1] = madgwick.getRollRadians();
	retbuf[2] = madgwick.getYawRadians();
}

void Motion_control::calib(){
	imu.calibrate(true);
}

void Motion_control::correctInitValue(uint16_t num_loop){
	ESP_LOGI(TAG, "calibrate START");
	//ジャイロセンサの初期値を取得
    //whileループで平均をとる
 	for (uint8_t i = 0; i < 3; i++)
	{
		g0[i] = 0;
		a0[i] = 0;
		m0[i] = 0;
	}

    for (uint16_t calibStep = 0; calibStep < num_loop; calibStep++)
    {
        imu.readGyro();
		imu.readAccel();
		imu.readMag();

		//g0[0] += imu.gx;
		//g0[1] += imu.gy;
		//g0[2] += imu.gz;

		//a0[0] += imu.ax;
		//a0[1] += imu.ay;
		//a0[2] += imu.az;

		//m0[0] += imu.mx;
		//m0[1] += imu.my;
		//m0[2] += imu.mz;
	}
	for (uint8_t i = 0; i<3; i++){
		g0[i] = g0[i]/num_loop;
		a0[i] = a0[i]/num_loop;
		m0[i] = m0[i]/num_loop;	
	}
	ESP_LOGI(TAG, "calibrate FINISH");
}