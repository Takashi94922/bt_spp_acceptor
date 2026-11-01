#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include <string.h>
#include "freertos/FreeRTOS.h"

struct PID {
	float Kp, Ki, Kd;
	float dt_cont;
	float integral;
	float target; // 目標値（セットポイント）
	float i_limit = 10.0f; // 積分項の出力の上限[%]（アンチワインドアップ）。打ち消したい一定のトルクは約3%
	float i_band = 5.0f * 0.0174533f; // [rad] 誤差がこれ以内のときだけ積分する（大きなずれはP・Dに任せる）
	// 直近の出力の内訳[%]（記録用）。PIDモード以外では0
	float last_p = 0.0f, last_i = 0.0f, last_d = 0.0f;
	// current: 現在の角度[rad]、rate: その角度の変化率[rad/s]（ジャイロから計算）
	float calculatePID(float current, float rate);
	void reset() { integral = 0.0f; }
	void clearTerms() { last_p = last_i = last_d = 0.0f; }
};
class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
public:
	Motion_control(){
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

		// PIDの出力（列: pitch roll yaw）をサーボ（行: u0=推力（未使用）, u1～u4）に配分する
		float KPIDsrc[] = {
			0,0,0,
			-1, 1, 1,
			-1, -1, 1,
			1, -1, 1,
			1, 1, 1,	
		};
		KPID = dspm::Mat(KPIDsrc, 5, 3);

		// PID制御用のインスタンス
		//吸い込み力と釣り合わせるためpitchだけちょっと後ろに傾ける
		//{Kp, Ki, Kd, dt_cont, integral, target}
		pitch_pid = {7.2f, 0.0f, 0.54f, 0.0f, 0.0f, -0.1f};
		roll_pid = {9.60f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f};
		yaw_pid = {.0f, .0f, 0.00f, 0.0f, 0.0f, 0.0f};
	}

	static constexpr float gravity_c = 9.80665;
	static constexpr float deg2rad = 0.0174533;
	static constexpr float rad2deg = 1.0/deg2rad;
	float dt_imu = 10E-3;
	float dt_cont = 1.0f;

	// 磁気センサはODR 80Hzなので、IMU周期ごとには読まずに間引く
	static constexpr float mag_odr = 80.0f;
	uint16_t mag_div = 1;   // 何回に1回磁気を読むか
	uint16_t mag_count = 0;

	// モーターの磁場で地磁気が大きく乱れる（10%で地磁気そのものより大きい）ので、
	// モーターが回っている間と止めた直後は地磁気を使わず、加速度とジャイロの6軸で推定する
	static constexpr float mag_resume_time = 1.0f; // [s] モーターが止まってからこの時間たったら地磁気を使う
	bool motor_running = false;    // 制御タスクが毎周期 setMotorRunning で設定する
	uint32_t mag_resume_count = 0;
	bool mag_in_use = true;        // 直近の推定で地磁気を使ったか（状態の返送用）

	float g_bias[3] = {0};   // ジャイロのバイアス[rad/s]（IMU座標、符号反転後）

	// Madgwickのβ。起動直後は大きくして加速度・磁気の向きに素早く合わせ、その後通常値に戻す
	static constexpr float beta_init = 2.0f;
	static constexpr float beta_run = 0.1f;
	static constexpr float beta_init_time = 1.0f; // [s]
	uint32_t beta_init_count = 0;

	// 起動時のジャイロバイアス測定
	static constexpr uint16_t gyro_calib_samples = 500;
	static constexpr float gyro_still_std = 1.0f * deg2rad; // これ以上ばらつくと「動いていた」とみなす[rad/s]
	dspm::Mat q = dspm::Mat(4, 1);     // IMU座標の姿勢クォータニオン（BLEで送る）
	dspm::Mat u = dspm::Mat(5, 1);     // 制御出力 u0=推力（未使用）, u1～u4=サーボ[%]
	// IMU座標のセンサ値（符号変換・バイアス補正の後）
	dspm::Mat a_imu = dspm::Mat(3, 1); // 加速度[m/s^2]
	dspm::Mat g_imu = dspm::Mat(3, 1); // 角速度[rad/s]
	dspm::Mat m_imu = dspm::Mat(3, 1); // 地磁気[gauss]
	//単位はrad
	float PRY_value[3] = {0};
	float PRY_offset[3] = {0};  // 初期角度オフセット
	float PRY_rate[3] = {0};    // PRY_value の変化率[rad/s]（ジャイロから計算、LPFなし）。PIDのD項に使う
	dspm::Mat KC, KPID;
	// BLE（Sensor_Telem）で送るセンサ値。IMU座標で符号変換・バイアス補正の後
	// [0..2] 加速度[m/s^2]、[3..5] ジャイロ[rad/s]、[6..8] 地磁気[gauss]
	float sensor_telem[9] = {0};
	PID pitch_pid, roll_pid, yaw_pid;
	uint8_t ControlMethod = 0; // 0: None, 1: KC, 2: PID
	uint8_t prev_method = 0;   // モードが変わったらPIDの内部状態をリセットする

	// ゲイン・目標値・モードはBLEのタスク（Core 0）から書き換えられ、制御タスク（Core 1）が読む。
	// 書き換えは下のset系関数を通し、calcUの計算中と重ならないようにする
	portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;

	void begin(float sampleFreq, float Control_freq, i2c_master_bus_handle_t bus_handle);
    void updateSensorTelem(); // sensor_telem を最新のセンサ値で埋める
    void update();
    void calcU();
	//単位はrad
    void getPRY(float *retbuf);
	void getPRYrate(float *retbuf);
	void getQuaternion(float* retbuf);
	void setPRYtarget(float pitch_target, float roll_target, float yaw_target);
	void setPIDTarget(uint8_t axis, float target);            // axis 0:pitch 1:roll 2:yaw
	void setPIDGain(uint8_t axis, uint8_t type, float value); // type 0:Kp 1:Ki 2:Kd
	void setControlMethod(uint8_t method);
	void setKC(const uint8_t *src, uint16_t first, uint16_t count); // KC.data[first..] にfloatをcount個
	// モニターへの読み戻し用（BLEのタスクから呼ぶ）
	void getPIDGains(float *out);   // pitch, roll, yaw の順に Kp, Ki, Kd（9個）
	void getPIDTargets(float *out); // pitch, roll, yaw の目標値[rad]（3個）
	void getKC(float *out);         // KC を行優先で 5×6 = 30個
	bool calibrateGyroBias(uint16_t samples);
	bool calibrateGyro(); // バイアスを測り直し、姿勢推定を素早く合わせ直す。静止していればtrue
	void gyroImu(float *out);
	void setMotorRunning(bool running) { motor_running = running; }
};