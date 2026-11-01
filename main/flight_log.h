// 制御周期での記録（ゲイン調整用）。RAMに数秒分ためてから、BLE（Log_Telem）でまとめて送る。
//
// タスクの分担:
//   - テレメトリタスク: 記録の開始（バッファの確保）と送信。バッファの確保・解放はこのタスクだけが行う
//   - 制御タスク: 記録中（RECORDING）だけ record() で1サンプルずつ書き込む
#pragma once

#include <stdint.h>
#include <stddef.h>

// 1サンプル（制御周期ごと）。int16 の固定小数点。スケールは下の FLIGHT_LOG_SCALE_*
struct FlightLogSample {
    int16_t throttle;   // スロットル [0.01%]
    int16_t target[3];  // 目標角 pitch roll yaw [1e-4 rad]
    int16_t angle[3];   // 姿勢角 pitch roll yaw [1e-4 rad]
    int16_t rate[3];    // 角速度 pitch roll yaw [1e-3 rad/s]
    int16_t pid[3][3];  // PIDの各項。軸（pitch roll yaw）ごとに P I D [0.01%]
    int16_t servo[4];   // サーボ出力 u1～u4 [0.01%]
};
static_assert(sizeof(FlightLogSample) == 46, "FlightLogSample size");

constexpr float FLIGHT_LOG_SCALE_PERCENT = 100.0f;
constexpr float FLIGHT_LOG_SCALE_ANGLE   = 10000.0f;
constexpr float FLIGHT_LOG_SCALE_RATE    = 1000.0f;

// Log_Telem の通知の種類（先頭1バイト）
enum FlightLogPacket : uint8_t {
    FLIGHT_LOG_PKT_HEADER = 0, // [0, 版, 周期us u16, サンプル数 u16, サンプルのバイト数 u8, 制御モード u8, PIDゲイン float×9, 目標値 float×3]
    FLIGHT_LOG_PKT_DATA   = 1, // [1, サンプル数 n u8, 先頭のサンプル番号 u16, サンプル×n]
    FLIGHT_LOG_PKT_END    = 2, // [2, サンプル数 u16]
};
constexpr uint8_t FLIGHT_LOG_VERSION = 1;

// 記録開始時の条件（ヘッダで送る）
struct FlightLogInfo {
    uint16_t period_us;
    uint8_t control_method;
    float pid_gains[9];  // pitch roll yaw の順に Kp Ki Kd
    float targets[3];    // pitch roll yaw [rad]
};

class FlightLog {
public:
    enum State : uint8_t { IDLE, RECORDING, READY };

    // 記録を始める（テレメトリタスクから）。確保できたサンプル数を返す（0なら失敗）
    uint16_t start(uint16_t samples, const FlightLogInfo &info);
    // 1サンプル書き込む（制御タスクから）。記録中でなければ何もしない
    void record(const FlightLogSample &s);
    bool isRecording() const { return state == RECORDING; }

    // 送信待ちのパケットがあるか（テレメトリタスクから）
    bool hasPacket() const;
    // 次のパケットを buf に作り、長さを返す。送れたら advance() を呼ぶ（失敗したら呼ばずに次の機会に送り直す）
    size_t buildPacket(uint8_t *buf, size_t max_len) const;
    void advance();
    // サンプル番号 first から count 個を送り直す。count == 0 ならヘッダから全部
    void requestResend(uint16_t first, uint16_t count);

    State getState() const { return state; }

private:
    FlightLogSample *buf = nullptr;
    uint16_t capacity = 0;
    volatile uint16_t count = 0;
    volatile State state = IDLE;
    FlightLogInfo info = {};

    // 送信の進み具合
    bool header_pending = false;
    bool end_pending = false;
    uint16_t send_next = 0;
    uint16_t send_end = 0;
    mutable uint8_t last_n = 0; // 直前の buildPacket で詰めたサンプル数
};
