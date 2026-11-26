#include "runtime.hpp"

#include "attitude.hpp"
#include "user_config.hpp"
#include "tx_api.h"
#include "chassis_control.hpp"
#include "mission.hpp"
#include "remote_input.hpp"
#include "vision.hpp"
#include "types.hpp"
#include "servo_control.hpp"

#include <cstdint>

namespace robot
{
namespace
{
namespace cfg = user_config::robot::control;

// 线程和栈需要持续存在。
TX_THREAD worker{};
alignas(8) uint8_t stack[cfg::stack_size]{};
CHAR thread_name[] = "control";
bool started = false;

// control 保存自己使用的最新姿态。
imu::attitude::state attitude{};
msg::subscriber<imu::attitude::state> attitude_sub;

Remote remote{};
Mission mission{};

vision::Error vision_error{};

void run(ULONG)
{
    const auto vision_status = vision::init();
    ULONG deadline = tx_time_get();

    for (;;)
    {
        deadline += cfg::period_ticks;

        // 有新姿态时更新，没有新数据时保留上一份。
        if constexpr (USER_AHRS_DEVICE != 0)
        {
            (void)attitude_sub.read(attitude);
        }

        remote_input::update(remote);

#if defined(STM32F407xx)
        const auto jog = remote_input::servo_jog(remote);
        servo_control::update(jog.target_us, jog.allow_move);
#endif

        const bool chassis_ready = chassis_control::poll_feedback();
        const bool drive_allowed = mission.allow_drive(remote, chassis_ready);
        if (!chassis_control::update(remote, drive_allowed))
        {
            mission.disarm_drive();
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
            tx_thread_sleep(cfg::period_ticks);
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

    auto status = remote_input::init();
    if (status != pnxtypes::status::ok)
    {
        return status;
    }

    status = chassis_control::init();
    if (status != pnxtypes::status::ok)
    {
        return status;
    }

    const UINT result = tx_thread_create(&worker, thread_name, run, 0, stack, sizeof(stack),
        cfg::priority, cfg::priority, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (result != TX_SUCCESS)
    {
        return pnxtypes::status::error;
    }

    started = true;
    return pnxtypes::status::ok;
}
} // namespace robot