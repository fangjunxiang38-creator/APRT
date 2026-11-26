#include "runtime.hpp"

#include "bmi088.hpp"
#include "dmimu.hpp"
#include "bsp_can.hpp"
#include "user_config.hpp"
#include "bsp_dwt.hpp"
#include "main.h"

extern "C" void app_start()
{

    auto result = bsp::dwt::init();
    if (result != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }

#if USER_AHRS_DEVICE == 1
    result = imu::start_bmi088();

#elif USER_AHRS_DEVICE == 2
    // 请求模式需要一把持续存在的 CAN 互斥锁。
    TX_MUTEX* bus_mutex = nullptr;

    if constexpr (user_config::dmimu.communication == imu::dmimu::mode::request)
    {
        static TX_MUTEX mutex;
        static CHAR name[] = "imu_can";

        if (tx_mutex_create(&mutex, name, TX_INHERIT) != TX_SUCCESS)
        {
            Error_Handler();
            return;
        }

        bus_mutex = &mutex;
    }

    result = imu::start_dmimu(bus_mutex);

    if (result == pnxtypes::status::ok)
    {
        result = bsp::can::init(user_config::dmimu_can);
    }

#elif USER_AHRS_DEVICE != 0
#error Unsupported AHRS device
#endif

    if (result != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }

    result = robot::start();

    if (result != pnxtypes::status::ok)
    {
        Error_Handler();
    }
}