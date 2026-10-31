#include "Motion_control.h"
#include <inttypes.h>
#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include "mat.h"

#define FLIGHT_MODE_VERT // 0:Vert 1:X
//#define FLIGHT_MODE_X // 0:KC 1:PID 2:Future reserved

// 3×1 行列から 3×3 のスキュー対称行列を生成
void Motion_control::skew( dspm::Mat &v )
{
	// v(i,0) は i 行目の要素
	skewM(0,0) =  0;         skewM(0,1) = -v(2,0);  skewM(0,2) =  v(1,0);
	skewM(1,0) =  v(2,0);    skewM(1,1) =  0;       skewM(1,2) = -v(0,0);
	skewM(2,0) = -v(1,0);    skewM(2,1) =  v(0,0);  skewM(2,2) =  0;
}

void Motion_control::begin(float sampleFreq, float Control_freq, i2c_master_bus_handle_t bus_handle){
 	if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == 0){
        ESP_LOGE(TAG, "imu initialize faile");
    }
	dt_imu = 1.0f / sampleFreq;
	dt_cont = 1.0f / Control_freq;
	pitch_pid.dt_cont = dt_cont;
	roll_pid.dt_cont = dt_cont;
	yaw_pid.dt_cont = dt_cont;

    // ---- LPF設定 ----
	alpha = 1.0f / (1.0f + 2.0f * 3.1415926f * fc * dt_imu); // LPFの係数計算

	mag_div = (uint16_t)(sampleFreq / mag_odr + 0.5f);
	if (mag_div < 1) mag_div = 1;
	mag_count = 0;

	madgwick.begin(sampleFreq);

	// 静止中にジャイロのバイアスを測ってから姿勢推定を始める
	calibrateGyro();
}

bool Motion_control::calibrateGyro(){
	bool still = calibrateGyroBias(gyro_calib_samples);
	madgwick.setBeta(beta_init);
	beta_init_count = (uint32_t)(beta_init_time / dt_imu);
	return still;
}

void Motion_control::Sensor2Body(){
	//imu座標から機体座標系
	//※LSM9DS1のg/aは左手系でmは右手系
	a = IMU_2_body * a_imu;
	g = IMU_2_body * g_imu;
	m = IMU_2_body * m_imu;

	skew(g);
	centripetal = skewM * (skewM * x_IMU);

	ga = ((g - g_prev) * (1.0f /dt_imu));
    skew(ga);
	tangential = skewM * x_IMU;
	//a = a - centripetal - tangential; //重心の座標系に変換

	g_prev = g;
}

void Motion_control::filterUpdate(){
	//Kalman Filterの更新処理だが、ESP32の性能を考慮して、計算はテレメトリ先で行う。
	xhat(0, 0) = a_imu(0, 0);
	xhat(1, 0) = a_imu(1, 0);
	xhat(2, 0) = a_imu(2, 0);
	xhat(3, 0) = g_imu(0, 0);
	xhat(4, 0) = g_imu(1, 0);
	xhat(5, 0) = g_imu(2, 0);	

	//以下はESP32で計算するばあいのコード。
	/* 	
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
    vx += acc_world(0,0) * dt_imu;
    vy += acc_world(1,0) * dt_imu;
    vz += acc_world(2,0) * dt_imu;

    px += vx * dt_imu + 0.5f * acc_world(0,0) * dt_imu * dt_imu;
    py += vy * dt_imu + 0.5f * acc_world(1,0) * dt_imu * dt_imu;
    pz += vz * dt_imu + 0.5f * acc_world(2,0) * dt_imu * dt_imu;

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
	Phi += F * dt_imu;

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
	//ESP_LOGI(TAG, "Kalman Updated"); */
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
	// I2Cトランザクションを減らすため、未使用の温度は読まず、磁気はmag_div回に1回だけ読む
    imu.readAccel();
    imu.readGyro();
	if (mag_count == 0) {
		imu.readMag();
	}
	if (++mag_count >= mag_div) mag_count = 0;

	//LSM9DS1のg/aは左手系でmは右手系なので、ここで座標変換を行う。
	a_imu(0, 0) = imu.calcAccel(imu.ax) * gravity_c;
	a_imu(1, 0) = -imu.calcAccel(imu.ay) * gravity_c;
	a_imu(2, 0) = imu.calcAccel(imu.az) * gravity_c;
	float g_raw[3];
	gyroImu(g_raw);
	g_imu(0, 0) = g_raw[0] - g_bias[0];
	g_imu(1, 0) = g_raw[1] - g_bias[1];
	g_imu(2, 0) = g_raw[2] - g_bias[2];
	m_imu(0, 0) = -imu.calcMag(imu.mx) * 0.00014f;
	m_imu(1, 0) = -imu.calcMag(imu.my) * 0.00014f;
	m_imu(2, 0) = imu.calcMag(imu.mz) * 0.00014f; // gauss/LSB

	//IMU座標系で姿勢を計算	
	madgwick.update(g_imu(0, 0), g_imu(1, 0), g_imu(2, 0),
				a_imu(0, 0), a_imu(1, 0), a_imu(2, 0),
				m_imu(0, 0), m_imu(1, 0), m_imu(2, 0));
	if (beta_init_count > 0 && --beta_init_count == 0) {
		madgwick.setBeta(beta_run);
	}

	//計算結果を取得（IMU座標系 → 機体座標系の補正はここで行う）
	getPRY(PRY_value);
	getPRYrate(PRY_rate);
	getQuaternion(q.data);

	//ESP_LOGI(TAG, "raw%1.2f,%1.2f,%1.2f", a(0, 0), a(1, 0), a(2, 0));
	//ESP_LOGI(TAG, "u%2.1f,%2.1f,%2.1f", u(1, 0), u(2, 0), u(3, 0));
}
// out = scale * M * x + offset をoutに直接書き込む
// dspm::Matの演算子は一時行列をヒープ確保するので、制御周期ごとの計算ではこちらを使う
static void mat_vec_affine(dspm::Mat &out, const dspm::Mat &M, const float *x, float scale, float offset)
{
	for (int i = 0; i < M.rows; i++) {
		float acc = 0.0f;
		for (int j = 0; j < M.cols; j++) {
			acc += M(i, j) * x[j];
		}
		out(i, 0) = scale * acc + offset;
	}
}

// u(1,0)～u(4,0)（サーボ）を lo～hi にクランプ
static void clamp_servo(dspm::Mat &u, float lo, float hi)
{
	for (uint8_t i = 1; i < 5; i++) {
		if (u(i, 0) < lo) u(i, 0) = lo;
		if (u(i, 0) > hi) u(i, 0) = hi;
	}
}

void Motion_control::calcU(){
	// ゲイン・目標値・モードの書き換え（BLEのタスク）と重ならないようにする
	taskENTER_CRITICAL(&param_lock);
	if (ControlMethod != prev_method) {
		pitch_pid.reset();
		roll_pid.reset();
		yaw_pid.reset();
		prev_method = ControlMethod;
	}

	if (ControlMethod == 0 || ControlMethod == 1) {
		// 状態は機体座標の姿勢角（PIDと共通の目標値との差）。x1～x3 と x6 は推定していないので0
		float xsrc[] = {0, 0, 0, PRY_value[0] - pitch_pid.target, PRY_value[1] - roll_pid.target, 0};
		#ifdef FLIGHT_MODE_X
			mat_vec_affine(u, KC, xsrc, -1.0f, 75.0f);
		#elif defined(FLIGHT_MODE_VERT)
			mat_vec_affine(u, KC, xsrc, -1.0f, 50.0f);
			clamp_servo(u, 0.0f, 100.0f);
		#endif
	}
	else if (ControlMethod == 2) {
		// PID制御を適用
		float xsrc[3];
		xsrc[0] = pitch_pid.calculatePID(PRY_value[0], PRY_rate[0]);
		xsrc[1] = roll_pid.calculatePID(PRY_value[1], PRY_rate[1]);
		xsrc[2] = yaw_pid.calculatePID(PRY_value[2], PRY_rate[2]);

		#ifdef FLIGHT_MODE_X
			// 制御出力を使用して次の処理を実行
			mat_vec_affine(u, KPID, xsrc, 1.0f, 75.0f);
			clamp_servo(u, 50.0f, 100.0f);
		#elif defined(FLIGHT_MODE_VERT)
			// V字のモードは、u(1,0)～u(4,0)を0～100の範囲にクランプ
			mat_vec_affine(u, KPID, xsrc, 1.0f, 50.0f);
			clamp_servo(u, 0.0f, 100.0f);
		#endif
	}
	else if(ControlMethod == 3){
		//future reserved
	}
	taskEXIT_CRITICAL(&param_lock);
}

// PID制御計算関数
float PID::calculatePID(float current, float rate) {
    float error = target - current; // 誤差
    // 微分項。目標値を一定とみなし d(error)/dt = -rate とする。
    // 角度の差分と違って遅れとノイズがなく、目標値を変えても出力が跳ねない
    float derivative = -rate;

    // 積分項。Ki*integral が ±i_limit を超えないように積分値を制限する（アンチワインドアップ）
    if (fabsf(Ki) > 1e-6f) {
        integral += error * dt_cont;
        float lim = i_limit / fabsf(Ki);
        if (integral > lim) integral = lim;
        if (integral < -lim) integral = -lim;
    } else {
        integral = 0.0f;
    }

    // PID制御出力
    return Kp * error + Ki * integral + Kd * derivative;
}

void Motion_control::getQuaternion(float* retbuf){
	retbuf[0] = madgwick.q0;
	retbuf[1] = madgwick.q1;
	retbuf[2] = madgwick.q2;
	retbuf[3] = madgwick.q3;
}

// PRY値を取得する関数単位はrad
void Motion_control::getPRY(float* retbuf){
	//IMU座標系から機体座標系の姿勢角を取得する
	//retbufの順番は pitch roll yaw
	//X字
/* 	retbuf[0] = -(madgwick.getRollRadians() - 1.5707963267f); // pitch
	retbuf[1] =  (madgwick.getPitchRadians() - PRY_offset[1]); // roll
	retbuf[2] = -(madgwick.getYawRadians() - PRY_offset[2]); // yaw */
	//V字
	/*
 	retbuf[0] = (madgwick.getRollRadians() - 1.5707963267f); // pitch
	retbuf[1] =  -(madgwick.getPitchRadians() - PRY_offset[1]); // roll
	retbuf[2] = -(madgwick.getYawRadians() - PRY_offset[2]); // yaw
*/

    // ---- 機体座標の姿勢角（V字配置）----
    // Madgwick側で q_body = q_imu ⊗ q_mount を計算済みなので、機体姿勢のオイラー角をそのまま使う。
    // 機体が大きく傾いてもロール・ピッチ・ヨーが混ざらない。
    // 符号はホバリング付近で従来の式（IMUのオイラー角を足し引きしていた）と一致させてある
    float body[3]; // roll pitch yaw
    madgwick.retBodyAngles(body);
    float yaw = body[2] + 1.5707963267f; // 従来と同じくIMUのヨー角を基準にする
    if (yaw > 3.1415926535f) yaw -= 6.2831853071f;
    float raw_pitch =  body[1] - PRY_offset[0];
    float raw_roll  =  body[0] - PRY_offset[1];
    float raw_yaw   = -yaw     - PRY_offset[2];

    // ---- LPF用の内部状態 ----
    static float flt_pitch = 0.0f;
    static float flt_roll  = 0.0f;
    static float flt_yaw   = 0.0f;

    // ---- 一次LPF ----
    flt_pitch = alpha * flt_pitch + (1.0f - alpha) * raw_pitch;
    flt_roll  = alpha * flt_roll  + (1.0f - alpha) * raw_roll;
    flt_yaw   = alpha * flt_yaw   + (1.0f - alpha) * raw_yaw;

    // ---- 出力 ----
    retbuf[0] = flt_pitch;
    retbuf[1] = flt_roll;
    retbuf[2] = flt_yaw;
}

// PRY_value の変化率[rad/s]をジャイロ（バイアス補正後）から求める。
// 機体座標の角速度（機体x=IMU z、機体y=IMU x、機体z=IMU y、IMU_2_body と同じ）を
// ZYXオイラー角の変化率に直し、getPRY と同じ並びと符号にする
void Motion_control::getPRYrate(float* retbuf){
	float p = g_imu(2, 0); // 機体x軸まわり
	float q = g_imu(0, 0); // 機体y軸まわり
	float r = g_imu(1, 0); // 機体z軸まわり

	float body[3]; // roll pitch yaw
	madgwick.retBodyAngles(body);
	float sr = sinf(body[0]), cr = cosf(body[0]);
	float cp = cosf(body[1]);
	if (fabsf(cp) < 0.01f) cp = (cp < 0.0f) ? -0.01f : 0.01f; // ピッチ±90°付近の発散を防ぐ
	float tp = sinf(body[1]) / cp;

	float roll_rate  = p + (q * sr + r * cr) * tp;
	float pitch_rate = q * cr - r * sr;
	float yaw_rate   = (q * sr + r * cr) / cp;

	retbuf[0] = pitch_rate;
	retbuf[1] = roll_rate;
	retbuf[2] = -yaw_rate; // getPRY のヨーは符号反転している
}

void Motion_control::setPRYtarget(float pitch_target, float roll_target, float yaw_target){
	taskENTER_CRITICAL(&param_lock);
	pitch_pid.target = pitch_target;
	roll_pid.target = roll_target;
	yaw_pid.target = yaw_target;
	taskEXIT_CRITICAL(&param_lock);
}

void Motion_control::setPIDTarget(uint8_t axis, float target){
	PID* pids[] = {&pitch_pid, &roll_pid, &yaw_pid};
	if (axis >= 3) return;
	taskENTER_CRITICAL(&param_lock);
	pids[axis]->target = target;
	taskEXIT_CRITICAL(&param_lock);
}

void Motion_control::setPIDGain(uint8_t axis, uint8_t type, float value){
	PID* pids[] = {&pitch_pid, &roll_pid, &yaw_pid};
	if (axis >= 3 || type >= 3) return;
	taskENTER_CRITICAL(&param_lock);
	switch (type) {
		case 0: pids[axis]->Kp = value; break;
		case 1: pids[axis]->Ki = value; break;
		case 2: pids[axis]->Kd = value; break;
	}
	taskEXIT_CRITICAL(&param_lock);
}

void Motion_control::setControlMethod(uint8_t method){
	taskENTER_CRITICAL(&param_lock);
	ControlMethod = method;
	taskEXIT_CRITICAL(&param_lock);
}

void Motion_control::setKC(const uint8_t *src, uint16_t first, uint16_t count){
	if (first + count > (uint16_t)(KC.rows * KC.cols)) return;
	taskENTER_CRITICAL(&param_lock);
	memcpy(&KC.data[first], src, sizeof(float) * count);
	taskEXIT_CRITICAL(&param_lock);
}

// ジャイロの角速度[rad/s]をIMU座標（Y軸の符号反転後）で取得する。バイアスは引かない
void Motion_control::gyroImu(float *out){
	out[0] = imu.calcGyro(imu.gx) * deg2rad;
	out[1] = -imu.calcGyro(imu.gy) * deg2rad;
	out[2] = imu.calcGyro(imu.gz) * deg2rad;
}

// 静止状態でジャイロのバイアスを測る。起動直後に呼ぶので、その間は機体を動かさないこと。
// 動いていたと判定したら測り直す（3回まで）。3回とも動いていたらfalse（最後の結果を使う）
bool Motion_control::calibrateGyroBias(uint16_t samples){
	for (int attempt = 0; attempt < 3; attempt++) {
		float sum[3] = {0}, sq[3] = {0};
		for (uint16_t i = 0; i < samples; i++) {
			imu.readGyro();
			float g_raw[3];
			gyroImu(g_raw);
			for (int k = 0; k < 3; k++) {
				sum[k] += g_raw[k];
				sq[k] += g_raw[k] * g_raw[k];
			}
			vTaskDelay(1); // 1ms（ジャイロのODRは952Hz）
		}

		float std_max = 0.0f;
		for (int k = 0; k < 3; k++) {
			g_bias[k] = sum[k] / samples;
			float var = sq[k] / samples - g_bias[k] * g_bias[k];
			float sd = var > 0.0f ? sqrtf(var) : 0.0f;
			if (sd > std_max) std_max = sd;
		}
		ESP_LOGI(TAG, "gyro bias [deg/s]: %.2f %.2f %.2f (std max %.2f)",
				 g_bias[0] * rad2deg, g_bias[1] * rad2deg, g_bias[2] * rad2deg, std_max * rad2deg);
		if (std_max <= gyro_still_std) return true;
		ESP_LOGW(TAG, "gyro calibration: drone seems to be moving, retry");
	}
	ESP_LOGW(TAG, "gyro calibration: using the last result although the drone was moving");
	return false;
}
