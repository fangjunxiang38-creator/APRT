#include "ps2_gpio.hpp"

#include "ps2.hpp"
#include "remoter.hpp"
#include "user_config.hpp"
#include "bsp_gpio.hpp"
#include "bsp_dwt.hpp"
#include "main.h"

namespace robot
{
bool Ps2Gpio::init(const bsp::gpio::config (&pins)[4], uint32_t bit_delay_us)
{
    initialized_ = false;

    if (bit_delay_us < 3 || bit_delay_us > 20)
    {
        return false;
    }

    for (unsigned i = 0; i < 4; ++i)
    {
        if (bsp::peripherals::gpio_port(pins[i].instance) == nullptr ||
            bsp::peripherals::gpio_pin(pins[i].pin) == 0)
        {
            return false;
        }

        for (unsigned j = 0; j < i; ++j)
        {
            if (pins[i].instance == pins[j].instance && pins[i].pin == pins[j].pin)
            {
                return false;
            }
        }
    }

    for (unsigned i = 0; i < 4; ++i)
    {
        pins_[i] = pins[i];
    }

    delay_us_ = bit_delay_us;
    setup_step_ = 0;
    missed_ = 0;
    initialized_ = true;
    return true;
}

bool Ps2Gpio::exchange(const uint8_t (&tx)[9], uint8_t (&rx)[9])
{
    using pnxtypes::status;

    bool ok = bsp::gpio::write(pins_[2], false) == status::ok;
    bsp::dwt::delay_us(20);

    for (unsigned i = 0; i < 9 && ok; ++i)
    {
        rx[i] = 0;

        // PS2 每字节从最低位开始传输。
        for (unsigned bit = 0; bit < 8 && ok; ++bit)
        {
            ok = bsp::gpio::write(pins_[1], ((tx[i] >> bit) & 1U) != 0) == status::ok &&
                 bsp::gpio::write(pins_[3], false) == status::ok;

            bsp::dwt::delay_us(delay_us_);

            ok = ok && bsp::gpio::write(pins_[3], true) == status::ok;

            bool high = false;
            ok = ok && bsp::gpio::read(pins_[0], high) == status::ok;

            if (high)
            {
                rx[i] |= static_cast<uint8_t>(1U << bit);
            }

            bsp::dwt::delay_us(delay_us_);
        }

        bsp::dwt::delay_us(20);
    }

    // 无论本轮是否成功，都尝试恢复空闲电平。
    const bool released = bsp::gpio::write(pins_[2], true) == status::ok;
    const bool clock_idle = bsp::gpio::write(pins_[3], true) == status::ok;
    const bool command_idle = bsp::gpio::write(pins_[1], true) == status::ok;

    return ok && released && clock_idle && command_idle;
}

bool Ps2Gpio::poll(Remote& result)
{
    if (!initialized_ || !remotes::enabled())
    {
        return false;
    }

    constexpr uint8_t setup[3][9]{{1, 0x43, 0, 1, 0, 0, 0, 0, 0}, {1, 0x44, 0, 1, 3, 0, 0, 0, 0},
        {1, 0x43, 0, 0, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A}};

    remotes::ps2::frame raw{};

    // 前三次轮询分别进入配置、设置模拟模式、退出配置。
    if (setup_step_ < 3)
    {
        if (exchange(setup[setup_step_], raw.bytes))
        {
            ++setup_step_;
        }

        return false;
    }

    constexpr uint8_t query[9]{1, 0x42, 0, 0, 0, 0, 0, 0, 0};
    remotes::ps2::sample decoded{};

    if (!exchange(query, raw.bytes) ||
        remotes::ps2::decode(raw, decoded, user_config::ps2_deadzone) != pnxtypes::status::ok)
    {
        if (++missed_ >= 3)
        {
            missed_ = 0;
            setup_step_ = 0;
        }

        return false;
    }

    missed_ = 0;

    if (!remotes::enabled())
    {
        return false;
    }

    // 将模板的布尔按键结构转换成 PS2 顺序的 16 位按键值。
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

    result = {decoded.lx, decoded.ly, decoded.rx, decoded.ry, buttons, HAL_GetTick(), true};

    return true;
}
} // namespace robot