#include "ps2_gpio.hpp"

#include "bsp_dwt.hpp"
#include "peripherals.hpp"

namespace remotes::ps2
{
bool gpio::init(const bsp::gpio::config (&pins)[4], uint32_t bit_delay_us)
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

bool gpio::exchange(const uint8_t (&tx)[frame_size], uint8_t (&rx)[frame_size])
{
    using pnxtypes::status;

    bool ok = bsp::gpio::write(pins_[2], false) == status::ok;
    bsp::dwt::delay_us(20);

    for (unsigned i = 0; i < frame_size && ok; ++i)
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

bool gpio::poll(sample& result, float deadzone)
{
    if (!initialized_)
    {
        return false;
    }

    constexpr uint8_t setup[3][frame_size]{{1, 0x43, 0, 1, 0, 0, 0, 0, 0},
        {1, 0x44, 0, 1, 3, 0, 0, 0, 0}, {1, 0x43, 0, 0, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A}};

    frame raw{};
    if (setup_step_ < 3)
    {
        if (exchange(setup[setup_step_], raw.bytes))
        {
            ++setup_step_;
        }
        return false;
    }

    constexpr uint8_t query[frame_size]{1, 0x42, 0, 0, 0, 0, 0, 0, 0};
    if (!exchange(query, raw.bytes) || decode(raw, result, deadzone) != pnxtypes::status::ok)
    {
        if (++missed_ >= 3)
        {
            missed_ = 0;
            setup_step_ = 0;
        }
        return false;
    }

    missed_ = 0;
    return true;
}
} // namespace remotes::ps2