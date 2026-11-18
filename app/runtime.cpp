#include "runtime.hpp"

#include "attitude.hpp"
#include "user_config.hpp"
#include "tx_api.h"
#include "ps2_gpio.hpp"
#include "robot_pins.hpp"
#include "bsp_dwt.hpp"
#include "remoter.hpp"
#include "vision.hpp"
#include "main.h"

#include <cstdint>

namespace robot
{
namespace
{
namespace cfg = user_config::robot;

// 线程和栈需要持续存在。
TX_THREAD worker{};
alignas(8) uint8_t stack[cfg::control_stack_size]{};
CHAR thread_name[] = "control";
bool started = false;

// control 保存自己使用的最新姿态。
imu::attitude::state attitude{};
msg::subscriber<imu::attitude::state> attitude_sub;

Ps2Gpio ps2{};
Remote remote{};

vision::Error vision_error{};

void run(ULONG)
{
    ULONG deadline = tx_time_get();
    uint32_t last_poll = HAL_GetTick() - cfg::ps2_period_ms;
    const auto vision_status = vision::init();

    for (;;)
    {
        deadline += cfg::control_period_ticks;

        // 有新姿态时更新，没有新数据时保留上一份。
        if constexpr (USER_AHRS_DEVICE != 0)
        {
            (void)attitude_sub.read(attitude);
        }

        if constexpr (cfg::ps2_enabled)
        {
            const uint32_t now = HAL_GetTick();

            if (uint32_t(now - last_poll) >= cfg::ps2_period_ms)
            {
                last_poll = now;
                (void)ps2.poll(remote);
            }

            // 收包可能跨过毫秒边界，使用收包完成后的当前时间判断。
            if (!remotes::enabled() ||
                !fresh(remote.valid, remote.received_ms, HAL_GetTick(), cfg::input_timeout_ms))
            {
                // 超时或禁止遥控时归中，保留最后一次成功接收的时间。
                const uint32_t last_received = remote.received_ms;
                remote = {};
                remote.received_ms = last_received;
            }
        }

        if (vision_status == pnxtypes::status::ok)
        {
            vision::update(vision_error, attitude);
        }

        // 按绝对截止时刻等待，减少处理耗时造成的周期漂移。
        const LONG remaining = static_cast<LONG>(deadline - tx_time_get());

        if (remaining > 0)
        {
            tx_thread_sleep(static_cast<ULONG>(remaining));
        }
        else
        {
            // 超过本轮周期后先让出 CPU，再建立新的时间基准。
            tx_thread_sleep(cfg::control_period_ticks);
            deadline = tx_time_get();
        }
    }
}
} // namespace

pnxtypes::status start()
{
    if (started)
    {
        return pnxtypes::status::busy;
    }

    // 姿态服务先启动，它负责初始化发布通道。
    if constexpr (USER_AHRS_DEVICE != 0)
    {
        if (!attitude_sub.valid())
        {
            attitude_sub = imu::attitude::output.subscribe();
        }

        if (!attitude_sub.valid())
        {
            return pnxtypes::status::not_initialized;
        }
    }

    if constexpr (cfg::ps2_enabled)
    {
        if (!ps2.init(user_config::robot_ps2_pins, cfg::ps2_bit_delay_us) ||
            !board::prepare_ps2_pins(user_config::robot_ps2_pins))
        {
            return pnxtypes::status::invalid_arg;
        }
    }

    const UINT result = tx_thread_create(&worker, thread_name, run, 0, stack, sizeof(stack),
        cfg::control_priority, cfg::control_priority, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (result != TX_SUCCESS)
    {
        return pnxtypes::status::error;
    }

    started = true;
    return pnxtypes::status::ok;
}
} // namespace robot