#include "AppMessageQueues.h"
#include "clock.h"
#include <connectivity/uart.h>
#include <Semaphore.h>
#include <critical_section.h>
#include <msgQueue.h>
#include <referee.h>
#include <sheriff_os.h>

// ---- 裁判系统数据
dji::DjiReferee referee_data;
extern int ui_self_id;
struct REFEREE_RAW_DATA {
    uint8_t buffer[500]{};
    uint16_t len{};
    REFEREE_RAW_DATA(uint8_t const* buf, uint16_t const _len) {
        memcpy(buffer, buf, _len);
        len = _len;
    }
    REFEREE_RAW_DATA() = default;
    ~REFEREE_RAW_DATA() = default;
};

// 这是一个线程安全的消息队列，因此不存储在AppMessageQueues.h中
os::MsgQueue<REFEREE_RAW_DATA, 3> referee_data_que;

InitLate(ConnectRefereeRXCallback) {
    os::Uart::Instance(USART2).on_receive.connect([&](uint8_t const* buff, uint16_t const len) {
        if (len > 500) {
            OsError("out of buffer range");
        }
        REFEREE_RAW_DATA make_data(buff, len);
        referee_data_que.Send_safe(&make_data);
    });
}

auto delta_tick = 0_ms;
auto last_update_tick = 0_ms;
REFEREE_RAW_DATA raw_data;
CREATE_THREAD_STATIC(refereeDataDecode, 512, NULL, 5) {
    for (;;) {
        if (referee_data_que.ReceiveTo(&raw_data, 300_ms)) {
            SECTION_SAFE_FROM_TASKS {
                delta_tick = os::GetTime() - last_update_tick;
                last_update_tick = os::GetTime();
                referee_data.Receive(raw_data.buffer, raw_data.len);
                auto power_limit = static_cast<float>(referee_data.robotPerf_.PowerLimit);
                ui_self_id = referee_data.robotPerf_.ID;
                os::referee_power_limit_queue.Send(&power_limit, 0_ms);
            }
        }
    }
}
