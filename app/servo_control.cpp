#include "servo_control.hpp"
#if defined(STM32F407xx)
#include "bsp_pwm.hpp"
#include "main.h"
#include "robot_pins.hpp"
#include "user_config.hpp"
#include <algorithm>

namespace robot::servo_control
{
namespace
{
namespace cfg = user_config::robot::servo;
using pnxtypes::status;
constexpr auto& timer = bsp::peripherals::tim1;
constexpr auto channel = bsp::pwm::channel::ch1;
constexpr uint32_t update_ms = 20;
constexpr float step_us = cfg::rate_us_s * (update_ms / 1000.0f);
float pulse_us = static_cast<float>(cfg::home_us);
uint32_t last_update_ms = 0;
bool started = false;
status pwm_status = status::ok;
} // namespace

void update(uint32_t target_us, bool allow_move)
{
    if (!allow_move || pwm_status != status::ok)
    {
        return;
    }

    const uint32_t now_ms = HAL_GetTick();
    if (uint32_t(now_ms - last_update_ms) < update_ms)
    {
        return;
    }
    last_update_ms = now_ms;

    if (!started)
    {
        pwm_status = bsp::pwm::set_pulse_width_us(timer, channel, cfg::home_us);
        if (pwm_status == status::ok)
        {
            board::latch_servo_pulses(timer);
            pwm_status = bsp::pwm::start(timer.instance, channel);
            started = pwm_status == status::ok;
        }
        return;
    }

    const float target = static_cast<float>(std::clamp(target_us, cfg::min_us, cfg::max_us));
    const float next_us = pulse_us + std::clamp(target - pulse_us, -step_us, step_us);
    if (next_us == pulse_us)
    {
        return;
    }

    pwm_status = bsp::pwm::set_pulse_width_us(timer, channel, static_cast<uint32_t>(next_us));
    if (pwm_status == status::ok)
    {
        pulse_us = next_us;
    }
}
} // namespace robot::servo_control
#endif