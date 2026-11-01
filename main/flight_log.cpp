#include "flight_log.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "FLIGHT_LOG";

// 記録用に確保しても残しておくヒープ[byte]。BLEなどの動作に必要な分
static constexpr size_t HEAP_RESERVE = 30 * 1024;

uint16_t FlightLog::start(uint16_t samples, const FlightLogInfo &new_info)
{
    if (state == RECORDING) return 0;
    state = IDLE;
    header_pending = end_pending = false;
    free(buf);
    buf = nullptr;
    capacity = 0;

    // 空きが足りなければ、入る分だけ記録する
    size_t free_bytes = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    size_t max_samples = free_bytes > HEAP_RESERVE ? (free_bytes - HEAP_RESERVE) / sizeof(FlightLogSample) : 0;
    if (samples > max_samples) samples = (uint16_t)max_samples;
    if (samples == 0) {
        ESP_LOGE(TAG, "not enough memory (largest free block %u bytes)", (unsigned)free_bytes);
        return 0;
    }
    buf = (FlightLogSample *)malloc(sizeof(FlightLogSample) * samples);
    if (buf == nullptr) {
        ESP_LOGE(TAG, "malloc failed (%u samples)", samples);
        return 0;
    }
    capacity = samples;
    count = 0;
    info = new_info;
    ESP_LOGI(TAG, "start recording %u samples", samples);
    state = RECORDING; // 最後に切り替える。ここから制御タスクが書き込む
    return samples;
}

void FlightLog::record(const FlightLogSample &s)
{
    if (state != RECORDING) return;
    buf[count] = s;
    count = count + 1;
    if (count >= capacity) {
        // 記録が終わったら、ヘッダから全部送る
        header_pending = end_pending = true;
        send_next = 0;
        send_end = capacity;
        state = READY;
    }
}

bool FlightLog::hasPacket() const
{
    return state == READY && (header_pending || send_next < send_end || end_pending);
}

size_t FlightLog::buildPacket(uint8_t *out, size_t max_len) const
{
    if (!hasPacket()) return 0;

    if (header_pending) {
        size_t len = 0;
        out[len++] = FLIGHT_LOG_PKT_HEADER;
        out[len++] = FLIGHT_LOG_VERSION;
        memcpy(&out[len], &info.period_us, 2); len += 2;
        memcpy(&out[len], &capacity, 2); len += 2;
        out[len++] = sizeof(FlightLogSample);
        out[len++] = info.control_method;
        memcpy(&out[len], info.pid_gains, sizeof(info.pid_gains)); len += sizeof(info.pid_gains);
        memcpy(&out[len], info.targets, sizeof(info.targets)); len += sizeof(info.targets);
        return len <= max_len ? len : 0;
    }

    if (send_next < send_end) {
        constexpr size_t head = 4;
        if (max_len < head + sizeof(FlightLogSample)) return 0;
        size_t n = (max_len - head) / sizeof(FlightLogSample);
        if (n > 255) n = 255;
        if (n > (size_t)(send_end - send_next)) n = send_end - send_next;
        out[0] = FLIGHT_LOG_PKT_DATA;
        out[1] = (uint8_t)n;
        memcpy(&out[2], &send_next, 2);
        memcpy(&out[head], &buf[send_next], n * sizeof(FlightLogSample));
        last_n = (uint8_t)n;
        return head + n * sizeof(FlightLogSample);
    }

    // end_pending
    out[0] = FLIGHT_LOG_PKT_END;
    memcpy(&out[1], &capacity, 2);
    return 3;
}

void FlightLog::advance()
{
    if (header_pending) {
        header_pending = false;
    } else if (send_next < send_end) {
        send_next += last_n;
    } else {
        end_pending = false;
    }
}

void FlightLog::requestResend(uint16_t first, uint16_t n)
{
    if (state != READY) return;
    if (n == 0) {
        header_pending = true;
        first = 0;
        n = capacity;
    }
    if (first >= capacity) return;
    if (n > capacity - first) n = capacity - first;
    send_next = first;
    send_end = first + n;
    end_pending = true;
}
