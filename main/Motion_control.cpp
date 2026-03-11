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
	//dt =  1.0f / sampleFreq; //sampleFreqはHzなので、dtは秒
	madgwick.begin(sampleFreq);
	//correctInitValue(10);
}

void Motion_control::Sensor2Body(){
	//imu座標から機体座標系
	//※LSM9DS1のg/aは左手系でmは右手系
	a_imu(0, 0) = imu.calcAccel(imu.ax) * gravity_c;
	a_imu(1, 0) = -imu.calcAccel(imu.ay) * gravity_c;
	a_imu(2, 0) = imu.calcAccel(imu.az) * gravity_c;
	a = IMU_2_body * a_imu;

	g_imu(0, 0) = imu.calcGyro(imu.gx) * deg2rad;
	g_imu(1, 0) = -imu.calcGyro(imu.gy) * deg2rad;
	g_imu(2, 0) = imu.calcGyro(imu.gz) * deg2rad;
	g = IMU_2_body * g_imu;

	m_imu(0, 0) = -imu.calcMag(imu.mx) * 0.00014f;
	m_imu(1, 0) = -imu.calcMag(imu.my) * 0.00014f;
	m_imu(2, 0) = imu.calcMag(imu.mz) * 0.00014f; // gauss/LSB
	m = IMU_2_body * m_imu;

	skew(g);
	centripetal = skewM * (skewM * x_IMU);

	ga = ((g - g_prev) * (1.0f /dt));
    skew(ga);
	tangential = skewM * x_IMU;
	a = a - centripetal - tangential; //重心の座標系に変換

	g_prev = g;
}
void Motion_control::filterUpdate(){
	//ESP_LOGD(TAG, "kalman Start");
	//x = F*x + B*u + Q
	//y = H*x
	
	//角速度の変化率
	madgwick.calcW();

	//xの代入
	//x = [vx, vy, vz, px, py , pz]';

	//まず q = [q0, q1, q2, q3]^T
	q(0, 0) = madgwick.qb0;
	q(1, 0) = madgwick.qb1;
	q(2, 0) = madgwick.qb2;
	q(3, 0) = madgwick.qb3;

    // 2) 世界座標系の加速度
    dspm::Mat acc_world(3,1);
    rotate(acc_world, q, a);      // a: 3x1 (body)
    acc_world(2,0) += 9.80665f;   // 座標系に応じて符号調整

    // 3) 状態の取り出し
    float vx = xhat(0,0);
    float vy = xhat(1,0);
    float vz = xhat(2,0);
    float px = xhat(3,0);
    float py = xhat(4,0);
    float pz = xhat(5,0);

    // 4) 予測更新
    vx += acc_world(0,0) * dt;
    vy += acc_world(1,0) * dt;
    vz += acc_world(2,0) * dt;

    px += vx * dt + 0.5f * acc_world(0,0) * dt * dt;
    py += vy * dt + 0.5f * acc_world(1,0) * dt * dt;
    pz += vz * dt + 0.5f * acc_world(2,0) * dt * dt;

    xhat(0,0) = vx;
    xhat(1,0) = vy;
    xhat(2,0) = vz;
    xhat(3,0) = px;
    xhat(4,0) = py;
    xhat(5,0) = pz;

	//F
	F.clear();

	// --- ṗ = v ---
	F(3,0) = 1.0f;
	F(4,1) = 1.0f;
	F(5,2) = 1.0f;
	
	//phi = I + F dt
	Phi = dspm::Mat::eye(6);
	Phi += F * dt;

	// --- P = Φ P Φᵀ + Q ---
	// temp = Φ * P
	temp = Phi * P;

	// P = temp * Φᵀ
	P = temp * Phi.t();

	// Q を加算（プロセスノイズ）
	P = P + Q;

	//y= [ax ay az];
	// 世界座標系の加速度に変換
	y(0, 0) = acc_world(0, 0);
	y(1, 0) = acc_world(1, 0);
	y(2, 0) = acc_world(2, 0);

	//H
	H.clear();
	// --- 加速度観測（0..2 行）---
	H(0, 0) = 0.00f;
	H(1, 1) = 0.0f;
	H(2, 2) = 0.0f;

	//K = P*H'*(R + H*P*H')^-1
	K = P*H.t()*(R + H*P*H.t()).inverse();

	//xhat = xhat + K(y-Hx)
	xhat += K*(y-H*xhat);

	//P = (I-KH)P
	P = (dspm::Mat::eye(6) - K*H)*P;
	//ESP_LOGI(TAG, "Kalman Updated");
}

// out = R(q) * a
// q: 4x1 (q0,q1,q2,q3)
// a: 3x1 (ax,ay,az)
// out: 3x1 (結果)

void Motion_control::rotate(dspm::Mat &out, const dspm::Mat &q, const dspm::Mat &a)
{
	float w = q(0,0);
	float x = q(1,0);
	float y = q(2,0);
	float z = q(3,0);

	float ax = a(0,0);
	float ay = a(1,0);
	float az = a(2,0);

	// 2*(q_vec × a)
	float t2x = 2.0f * ( y*az - z*ay );
	float t2y = 2.0f * ( z*ax - x*az );
	float t2z = 2.0f * ( x*ay - y*ax );

	// out = a + w*t2 + q_vec × t2
	out(0,0) = ax + w*t2x + ( y*t2z - z*t2y );
	out(1,0) = ay + w*t2y + ( z*t2x - x*t2z );
	out(2,0) = az + w*t2z + ( x*t2y - y*t2x );
}

void Motion_control::update(){
	imu.readTemp();
    imu.readAccel();
    imu.readGyro();
    imu.readMag();

	//IMU座標系で姿勢を計算
	//※LSM9DS1のg/aは左手系でmは右手系
	madgwick.update(imu.calcGyro(imu.gx), -imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
				imu.calcAccel(imu.ax), -imu.calcAccel(imu.ay), imu.calcAccel(imu.az),
				-imu.calcMag(imu.mx), -imu.calcMag(imu.my), imu.calcMag(imu.mz));

	//計算結果を取得（IMU座標系 → 機体座標系の補正はここで行う）
	getPRY(PRY_value);

	//imu->機体座標の変換＋向心力の補正
	Sensor2Body();

	//姿勢から重力の分力を減算
	//地球からbody座標に変換
	float gv_imusrc[3];
	madgwick.trans2body(gv_imusrc, gv);
	a_grav(0, 0) = gv_imusrc[0];
	a_grav(1, 0) = gv_imusrc[1];
	a_grav(2, 0) = gv_imusrc[2];
	//imuから機体に変換
	//a_grav = IMU_2_body * gv_imu;
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
		xsrc[0] = pitch_pid.calculatePID(PRY_value[0], dt);
		xsrc[1] = roll_pid.calculatePID(PRY_value[1], dt);
		xsrc[2] = yaw_pid.calculatePID(PRY_value[2], dt);

    	// 制御出力を使用して次の処理を実行
    	u = KPID * dspm::Mat(xsrc, 3, 1) + 75.0f;
		
		// u(1,0)～u(4,0)を50～100の範囲にクランプ
		for (uint8_t i = 1; i < 5; i++) {
			if (u(i, 0) < 50.0f) u(i, 0) = 50.0f;
			if (u(i, 0) > 100.0f) u(i, 0) = 100.0f;
		}
	}
	else if(ControlMethod == 3){
		//future reserved
	}
}

// PID制御計算関数
float PID::calculatePID(float current, float dt) {
    float error = target - current; // 誤差
    integral += error * dt;     // 積分項
    float derivative = (error - prev_error) / dt; // 微分項
    prev_error = error;         // 前回の誤差を更新

    // PID制御出力
    return Kp * error + Ki * integral + Kd * derivative;
}

// PRY値を取得する関数単位はrad
void Motion_control::getPRY(float* retbuf){
	//IMU座標系から機体座標系の姿勢角を取得する
	//retbufの順番は pitch roll yaw
	retbuf[0] = -(madgwick.getRollRadians() - 1.5707963267f); // pitch
	retbuf[1] =  (madgwick.getPitchRadians() - PRY_offset[1]); // roll
	retbuf[2] = -(madgwick.getYawRadians() - PRY_offset[2]); // yaw
}

void Motion_control::calib(){
	imu.calibrate(true);
}

void Motion_control::correctInitValue(uint16_t num_loop){
	ESP_LOGI(TAG, "calibrate START");
	//ジャイロセンサの初期値を取得
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