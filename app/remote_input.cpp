
#include "remote_input.hpp"

#include "main.h"
#include "ps2_gpio.hpp"
#include "remoter.hpp"
#include "robot_pins.hpp"
#include "user_config.hpp"

namespace robot::remote_input
{
namespace
{
namespace cfg = user_config::robot::remote;

remotes::ps2::gpio ps2{};
uint32_t last_poll_ms = 0;

void poll(Remote& remote)
{
    if (!remotes::enabled())
    {
        return;
    }

    remotes::ps2::sample decoded{};
    if (!ps2.poll(decoded, cfg::ps2::deadzone) || !remotes::enabled())
    {
        return;
    }

    const auto& key = decoded.buttons;
    const bool pressed[16]{key.select, key.l3, key.r3, key.start, key.up, key.right, key.down,
        key.left, key.l2, key.r2, key.l1, key.r1, key.triangle, key.circle, key.cross, key.square};

    uint16_t buttons = 0;
    for (unsigned i = 0; i < 16; ++i)
    {
        if (pressed[i])
        {
            buttons |= static_cast<uint16_t>(1U << i);
        }
    }

    remote = {decoded.lx, decoded.ly, decoded.rx, decoded.ry, buttons, HAL_GetTick(), true};
}
} // namespace

pnxtypes::status init()
{
    if constexpr (user_config::ps2_gpio_enabled)
    {
        if (!ps2.init(user_config::ps2_gpio_pins, cfg::ps2::bit_delay_us) ||
            !board::prepare_ps2_pins(user_config::ps2_gpio_pins))
        {
            return pnxtypes::status::invalid_arg;
        }
        last_poll_ms = HAL_GetTick() - cfg::ps2::poll_period_ms;
    }

    return pnxtypes::status::ok;
}

void update(Remote& remote)
{
    if constexpr (user_config::ps2_gpio_enabled)
    {
        const uint32_t now_ms = HAL_GetTick();
        if (uint32_t(now_ms - last_poll_ms) >= cfg::ps2::poll_period_ms)
        {
            last_poll_ms = now_ms;
            poll(remote);
        }

        // 轮询可能跨过毫秒边界，使用收包完成后的时刻。
        if (remotes::enabled() &&
            fresh(remote.valid, remote.received_ms, HAL_GetTick(), cfg::timeout_ms))
        {
            return;
        }

        const uint32_t last_received_ms = remote.received_ms;
        remote = {};
        remote.received_ms = last_received_ms;
    }
}

ServoJog servo_jog(const Remote& remote)
{
    namespace servo_cfg = user_config::robot::servo;
    constexpr uint16_t l2_mask = 1U << 8;
    constexpr uint16_t up_mask = 1U << 4;
    constexpr uint16_t down_mask = 1U << 6;
    static bool l2_released = false;

    const bool l2 = (remote.buttons & l2_mask) != 0U;
    const bool up = (remote.buttons & up_mask) != 0U;
    const bool down = (remote.buttons & down_mask) != 0U;

    if (!remote.valid || !l2)
    {
        l2_released = remote.valid;
    }

    const bool allow_move = remote.valid && l2_released && l2 && (up != down);
    return {up ? servo_cfg::max_us : servo_cfg::min_us, allow_move};
}
} // namespace robot::remote_input
