#include "Motion_control.h"
#include <inttypes.h>
#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include "mat.h"

// 3×1 行列から 3×3 のスキュー対称行列を生成
void Motion_control::skew( dspm::Mat &v )
{
// v(i,0) は i 行目の要素
skewM(0,0) =  0;         skewM(0,1) = -v(2,0);  skewM(0,2) =  v(1,0);
skewM(1,0) =  v(2,0);    skewM(1,1) =  0;       skewM(1,2) = -v(0,0);
skewM(2,0) = -v(1,0);    skewM(2,1) =  v(0,0);  skewM(2,2) =  0;
}

void Motion_control::begin(float sampleFreq, i2c_master_bus_handle_t bus_handle){
 	if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == 0){
        ESP_LOGE(TAG, "imu initialize faile");
    }
	dt =  1.0f / sampleFreq; //sampleFreqはHzなので、dtは秒
	madgwick.begin(sampleFreq);
}

void Motion_control::Sensor2Body(){
	//imu座標から機体座標系
	a_imu(0, 0) = imu.calcAccel(imu.ax) * gravity_c;
	a_imu(1, 0) = imu.calcAccel(imu.ay) * gravity_c;
	a_imu(2, 0) = imu.calcAccel(imu.az) * gravity_c;
	a = IMU_2_body * a_imu;

	g_imu(0, 0) = imu.calcGyro(imu.gx) * deg2rad;
	g_imu(1, 0) = imu.calcGyro(imu.gy) * deg2rad;
	g_imu(2, 0) = imu.calcGyro(imu.gz) * deg2rad;
	g = IMU_2_body * g_imu;

	m_imu(0, 0) = imu.calcMag(imu.mx - m0[0]) * 0.00014f;
	m_imu(1, 0) = imu.calcMag(imu.my - m0[1]) * 0.00014f;
	m_imu(2, 0) = imu.calcMag(imu.mz - m0[2]) * 0.00014f; // gauss/LSB
	m = IMU_2_body_mag * m_imu;

	skew(g);
	centripetal = skewM * (skewM * x_IMU);

	ga = ((g - g_prev) * (1.0f /dt));
    skew(ga);
	tangential = skewM * x_IMU;
	a = a - centripetal - tangential; //重心の座標系に変換

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

	//IMU座標系で姿勢を計算
	madgwick.update(imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
				imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az),
				imu.calcMag(imu.mx - m0[0]), imu.calcMag(imu.my - m0[1]), imu.calcMag(imu.mz - m0[2]));

	//計算結果を取得（IMU座標系 → 機体座標系の補正はここで行う）
	getPRY(PRY_value);

	//imu->機体座標の変換＋向心力の補正
	Sensor2Body();

	//姿勢から重力の分力を減算
	//地球からimu座標に変換
	float gv_imusrc[3];
	madgwick.trans(gv_imusrc, gv);
	gv_imu(0, 0) = gv_imusrc[0];
	gv_imu(1, 0) = gv_imusrc[1];
	gv_imu(2, 0) = gv_imusrc[2];
	//imuから機体に変換
	a_grav = IMU_2_body * gv_imu;
	//重力加速度を引く
	a = a - a_grav;

	//filtaUpdate();

	//ESP_LOGI(TAG, "raw%1.2f,%1.2f,%1.2f", a(0, 0), a(1, 0), a(2, 0));
	//ESP_LOGI(TAG, "u%2.1f,%2.1f,%2.1f", u(1, 0), u(2, 0), u(3, 0));
}
void Motion_control::calcU(){
	if (ControlMethod == 0 || ControlMethod == 1) {
		float xsrc[] = {0, 0, 0, madgwick.getPitchRadians(), madgwick.getRollRadians(), 0};
		u = -1 * KC * dspm::Mat(xsrc, 6, 1) + 75.0f;
	}
	else if (ControlMethod == 2) {
		// PID制御を適用
		float xsrc[3];
		xsrc[0] = calculatePID(pitch_pid, PRY_value[0]);
		xsrc[1] = calculatePID(roll_pid, PRY_value[1]);
		xsrc[2] = calculatePID(yaw_pid, PRY_value[2]);

    	// 制御出力を使用して次の処理を実行
    	u = KPID * dspm::Mat(xsrc, 3, 1) + 75.0f;
	}
	else if(ControlMethod == 3){
		//padの象限によって制御対象を変える
		float c45 = 0.70710678f;
		float dx = KPID(1, 0)*PRY_value[1];
		float dy = KPID(0, 0)*PRY_value[0];
		float dz = KPID(3, 0)*PRY_value[3];
		u(1, 0) = c45 * dx + c45 * dy < 0 ? 0 : (c45 * dx + c45 * dy) * 50;
		u(2, 0) = c45 * dx - c45 * dy < 0 ? 0 : (c45 * dx - c45 * dy) * 50;
		u(3, 0) = -c45 * dx - c45 * dy < 0 ? 0 : (-c45 * dx - c45 * dy) * 50;
		u(4, 0) = -c45 * dx + c45 * dy < 0 ? 0 : (-c45 * dx + c45 * dy) * 50;
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

// PRY値を取得する関数単位はrad
void Motion_control::getPRY(float* retbuf){
	//まずIMU座標系から機体座標系の姿勢角を取得する
	retbuf[0] = -madgwick.getPitchRadians();
	retbuf[1] = -madgwick.getRollRadians();
	retbuf[2] = -madgwick.getYawRadians();
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
		for (uint8_t i = 0; i < 3; i++)
		{
			g0[i] += imu.gx;
			a0[i] += imu.ax;
			m0[i] += imu.mx;
		}
	}
	for (uint8_t i = 0; i<3; i++){
		g0[i] = g0[i]/num_loop;
		a0[i] = a0[i]/num_loop;
		m0[i] = m0[i]/num_loop;	
	}
	ESP_LOGI(TAG, "calibrate FINISH");
}