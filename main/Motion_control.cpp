#include "Motion_control.h"
#include <inttypes.h>
#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include "mat.h"

static const char *TAG = "MOTION";

void Motion_control::begin(float sampleFreq, float Control_freq, i2c_master_bus_handle_t bus_handle){
 	if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == 0){
        ESP_LOGE(TAG, "imu initialize faile");
    }
	dt_imu = 1.0f / sampleFreq;
	dt_cont = 1.0f / Control_freq;
	pitch_pid.dt_cont = dt_cont;
	roll_pid.dt_cont = dt_cont;
	yaw_pid.dt_cont = dt_cont;

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

// テレメトリで送るセンサ値（IMU座標、符号変換・バイアス補正の後）
void Motion_control::updateSensorTelem(){
	for (int i = 0; i < 3; i++) {
		sensor_telem[i] = a_imu(i, 0);
		sensor_telem[3 + i] = g_imu(i, 0);
		sensor_telem[6 + i] = m_imu(i, 0);
	}
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
	// [gauss]。Madgwick は正規化して使うので、大きさは姿勢に影響しない
	m_imu(0, 0) = -imu.calcMag(imu.mx);
	m_imu(1, 0) = -imu.calcMag(imu.my);
	m_imu(2, 0) = imu.calcMag(imu.mz);

	if (motor_running) {
		mag_resume_count = (uint32_t)(mag_resume_time / dt_imu);
	} else if (mag_resume_count > 0) {
		mag_resume_count--;
	}
	mag_in_use = (mag_resume_count == 0);

	//IMU座標系で姿勢を計算	
	if (mag_in_use) {
		madgwick.update(g_imu(0, 0), g_imu(1, 0), g_imu(2, 0),
					a_imu(0, 0), a_imu(1, 0), a_imu(2, 0),
					m_imu(0, 0), m_imu(1, 0), m_imu(2, 0));
	} else {
		madgwick.updateIMU(g_imu(0, 0), g_imu(1, 0), g_imu(2, 0),
					a_imu(0, 0), a_imu(1, 0), a_imu(2, 0));
	}
	if (beta_init_count > 0 && --beta_init_count == 0) {
		madgwick.setBeta(beta_run);
	}

	//計算結果を取得（IMU座標系 → 機体座標系の補正はここで行う）
	getPRY(PRY_value);
	getPRYrate(PRY_rate);
	getQuaternion(q.data);
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
	// モーター停止中（地上）は積分しない。地面の傾きの分がたまり、離陸の瞬間に舵が入るのを防ぐ
	if (!motor_running) {
		pitch_pid.reset();
		roll_pid.reset();
		yaw_pid.reset();
	}

	if (ControlMethod != 2) {
		pitch_pid.clearTerms();
		roll_pid.clearTerms();
		yaw_pid.clearTerms();
	}

	if (ControlMethod == 0 || ControlMethod == 1) {
		// 状態は機体座標の姿勢角（PIDと共通の目標値との差）。x1～x3 と x6 は推定していないので0
		float xsrc[] = {0, 0, 0, PRY_value[0] - pitch_pid.target, PRY_value[1] - roll_pid.target, 0};
		mat_vec_affine(u, KC, xsrc, -1.0f, 50.0f);
		clamp_servo(u, 0.0f, 100.0f);
	}
	else if (ControlMethod == 2) {
		// PID制御を適用
		float xsrc[3];
		xsrc[0] = pitch_pid.calculatePID(PRY_value[0], PRY_rate[0]);
		xsrc[1] = roll_pid.calculatePID(PRY_value[1], PRY_rate[1]);
		xsrc[2] = yaw_pid.calculatePID(PRY_value[2], PRY_rate[2]);

		// サーボは中立50%を中心に、u(1,0)～u(4,0)を0～100の範囲にクランプ
		mat_vec_affine(u, KPID, xsrc, 1.0f, 50.0f);
		clamp_servo(u, 0.0f, 100.0f);
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

    // 積分項。Ki*integral が ±i_limit を超えないように積分値を制限する（アンチワインドアップ）。
    // 誤差が i_band を超えている間は積分を止めて値を保つ（手で押さえる・突風などで大きなずれが続いても積もらない）
    if (fabsf(Ki) > 1e-6f) {
        if (fabsf(error) <= i_band) integral += error * dt_cont;
        float lim = i_limit / fabsf(Ki);
        if (integral > lim) integral = lim;
        if (integral < -lim) integral = -lim;
    } else {
        integral = 0.0f;
    }

    // PID制御出力
    last_p = Kp * error;
    last_i = Ki * integral;
    last_d = Kd * derivative;
    return last_p + last_i + last_d;
}

void Motion_control::getQuaternion(float* retbuf){
	retbuf[0] = madgwick.q0;
	retbuf[1] = madgwick.q1;
	retbuf[2] = madgwick.q2;
	retbuf[3] = madgwick.q3;
}

// PRY値を取得する関数単位はrad
void Motion_control::getPRY(float* retbuf){
	//retbufの順番は pitch roll yaw

    // ---- 機体座標の姿勢角（V字配置）----
    // Madgwick側で q_body = q_imu ⊗ q_mount を計算済みなので、機体姿勢のオイラー角をそのまま使う。
    // 機体が大きく傾いてもロール・ピッチ・ヨーが混ざらない。
    // 符号はホバリング付近で従来の式（IMUのオイラー角を足し引きしていた）と一致させてある
    float body[3]; // roll pitch yaw
    madgwick.retBodyAngles(body);
    float yaw = body[2] + 1.5707963267f; // 従来と同じくIMUのヨー角を基準にする
    if (yaw > 3.1415926535f) yaw -= 6.2831853071f;
    // LPFはかけない（遅れを増やさないため。ノイズはMadgwick側で十分に落ちている）
    retbuf[0] =  body[1] - PRY_offset[0]; // pitch
    retbuf[1] =  body[0] - PRY_offset[1]; // roll
    retbuf[2] = -yaw     - PRY_offset[2]; // yaw
}

// PRY_value の変化率[rad/s]をジャイロ（バイアス補正後）から求める。
// 機体座標の角速度（機体x=IMU z、機体y=IMU x、機体z=IMU y、Madgwick の取付クォータニオンと同じ）を
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

void Motion_control::getPIDGains(float *out){
	const PID* pids[] = {&pitch_pid, &roll_pid, &yaw_pid};
	taskENTER_CRITICAL(&param_lock);
	for (int i = 0; i < 3; i++) {
		out[i * 3 + 0] = pids[i]->Kp;
		out[i * 3 + 1] = pids[i]->Ki;
		out[i * 3 + 2] = pids[i]->Kd;
	}
	taskEXIT_CRITICAL(&param_lock);
}

void Motion_control::getPIDTargets(float *out){
	taskENTER_CRITICAL(&param_lock);
	out[0] = pitch_pid.target;
	out[1] = roll_pid.target;
	out[2] = yaw_pid.target;
	taskEXIT_CRITICAL(&param_lock);
}

void Motion_control::getKC(float *out){
	taskENTER_CRITICAL(&param_lock);
	memcpy(out, KC.data, sizeof(float) * KC.rows * KC.cols);
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
