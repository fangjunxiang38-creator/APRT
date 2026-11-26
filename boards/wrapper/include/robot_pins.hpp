#pragma once

#include "config.hpp"
#include "peripherals.hpp"
#include "main.h"

namespace board
{
// 调用前已完成 MX_GPIO_Init，当前模板已开启各可用端口时钟。
// pins 顺序为 DATA、CMD、CS、CLK。
inline bool prepare_ps2_pins(const bsp::gpio::config (&pins)[4])
{
    for (unsigned i = 0; i < 4; ++i)
    {
        auto* port = bsp::peripherals::gpio_port(pins[i].instance);
        const auto mask = bsp::peripherals::gpio_pin(pins[i].pin);

        if (port == nullptr || mask == 0)
        {
            return false;
        }

        // 先预装空闲高电平，再切换为输出，减少启动毛刺。
        if (i != 0)
        {
            HAL_GPIO_WritePin(port, mask, GPIO_PIN_SET);
        }

        GPIO_InitTypeDef init{};
        init.Pin = mask;
        init.Mode = i == 0 ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT_PP;
        init.Pull = GPIO_PULLUP;
        init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(port, &init);
    }

    return true;
}

inline void latch_servo_pulses(const bsp::pwm::config& timer)
{
    bsp::peripherals::tim_handle(timer.instance)->Instance->EGR = TIM_EGR_UG;
}
} // namespace board