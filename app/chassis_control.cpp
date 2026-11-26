#include "chassis_control.hpp"

#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "chassis.hpp"
#include "dji_manager.hpp"
#include "m2006.hpp"
#include "main.h"
#include "remoter.hpp"
#include "user_config.hpp"

#include <array>

namespace robot::chassis_control
{
namespace cfg = user_config::robot::chassis;

static constexpr auto can = user_config::chassis_can;
static constexpr float dt =
    static_cast<float>(user_config::robot::control::period_ticks) / TX_TIMER_TICKS_PER_SECOND;
static constexpr Geometry geometry{cfg::wheel_radius_m, cfg::half_length_m, cfg::half_width_m};

struct Wheel
{
    motor::dji::m2006 device{};
    uint32_t received_ms = 0;
    bool valid = false;
};

// 左前、右前、左后、右后。
static Wheel wheels[4]{};
static motor::dji::manager motors{};
static Motion commanded{};
static std::array<float, 4> targets{};

pnxtypes::status init()
{
    if constexpr (can.instance == bsp::can::handle::none)
    {
        return pnxtypes::status::ok;
    }

    for (unsigned i = 0; i < 4; ++i)
    {
        auto& device = wheels[i].device;
        const auto feedback_id = static_cast<uint16_t>(0x200U + cfg::motor_ids[i]);
        if (!motors.register_motor(device, can.instance, feedback_id))
        {
            return pnxtypes::status::error;
        }

        device.speed_pid.tune(cfg::speed_kp, cfg::speed_ki, cfg::speed_kd);
        device.speed_pid.max_out = cfg::current_limit;
    }

    motors.set_period(dt);
    return bsp::can::init(can);
}

static bool update_targets(const Remote& remote, bool allowed)
{
    if (allowed)
    {
        const Motion target{
            remote.ly * cfg::max_vx,
            -remote.lx * cfg::max_vy,
            -remote.rx * cfg::max_wz,
        };
        commanded = ramp(commanded, target, cfg::linear_accel, cfg::angular_accel, dt);
        if (wheel_speeds(commanded, geometry, cfg::max_wheel_speed, targets))
        {
            return true;
        }
    }

    commanded = {};
    targets = {};
    return false;
}

bool poll_feedback()
{
    if constexpr (can.instance == bsp::can::handle::none)
    {
        return false;
    }

    for (auto& wheel : wheels)
    {
        if (motors.update_feedback(wheel.device, &wheel.received_ms))
        {
            wheel.valid = true;
        }
    }

    const uint32_t now_ms = HAL_GetTick();
    bool feedback_ready = true;
    for (auto& wheel : wheels)
    {
        wheel.valid = fresh(wheel.valid, wheel.received_ms, now_ms, cfg::feedback_timeout_ms);
        feedback_ready = feedback_ready && wheel.valid;
    }

    return cfg::outputs_enabled && feedback_ready &&
           bsp::can::current_status(can.instance) == pnxtypes::status::ok;
}

bool update(const Remote& remote, bool drive_allowed)
{
    if constexpr (can.instance == bsp::can::handle::none)
    {
        return !drive_allowed;
    }

    bool drive = false;
    if constexpr (cfg::outputs_enabled)
    {
        drive = update_targets(remote, drive_allowed);
        for (unsigned i = 0; i < 4; ++i)
        {
            auto& device = wheels[i].device;
            if (!drive && device.motor_mode == motor::mode::vel)
            {
                device.speed_pid.clear();
            }
            device.motor_mode = drive ? motor::mode::vel : motor::mode::relax;
            device.cmd.velocity = targets[i] * cfg::motor_directions[i];
        }
    }
    else
    {
        (void)update_targets(remote, remote.valid && remotes::enabled());
    }

    motors.update();
    if constexpr (cfg::outputs_enabled)
    {
        motors.transmit(static_cast<uint32_t>(bsp::dwt::timeline_us()));
    }

    return !drive_allowed || drive;
}
} // namespace robot::chassis_control