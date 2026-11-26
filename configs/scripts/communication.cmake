# USB 与 UI 参数。
config_bool(_usb_enabled usb enabled)
config_integer(_ui_timeout 0 2147483647 ui timeout_ms)

# 视觉串口波特率、姿态回传周期和收发数据有效期。时间单位为 ms。
config_integer(_vision_baud 9600 921600 vision baud)
config_integer(_vision_imu_period_ms 1 1000 vision imu_period_ms)
config_integer(_vision_imu_timeout_ms 10 1000 vision imu_timeout_ms)
config_integer(_vision_position_timeout_ms 10 1000 vision position_timeout_ms)

# 8N1 下 30 字节占 300 bit。周期按毫秒计，比较时将帧位数乘以 1000。
math(EXPR _vision_tx_budget "${_vision_baud} * ${_vision_imu_period_ms}")
if(_vision_tx_budget LESS_EQUAL 300000)
    message(FATAL_ERROR "vision.imu_period_ms must allow time to send one attitude frame")
endif()

# RFID read-only service parameters.
config_integer(_rfid_address 0 255 rfid address)
config_integer(_rfid_block 1 62 rfid block)
math(EXPR _rfid_trailer "${_rfid_block} % 4")
if(_rfid_trailer EQUAL 3)
    message(FATAL_ERROR "rfid.block: expected an ordinary S50 data block")
endif()
config_integer(_rfid_baud 1200 1000000 rfid baud)
config_integer(_rfid_priority 1 31 rfid priority)
config_integer(_rfid_stack_size 2048 16384 rfid stack_size)
foreach(_key IN ITEMS startup_ms poll_ms timeout_ms recovery_ms buzzer_period_us buzzer_on_ms buzzer_gap_ms)
    config_integer(_rfid_${_key} 1 60000 rfid ${_key})
endforeach()

# 裁判接收与应用遥控处理的调度参数。
config_integer(_remoter_stack 1024 16384 remoter stack_size)
config_integer(_referee_stack 2048 16384 referee stack_size)

foreach(_module IN ITEMS referee remoter)
    config_integer(_${_module}_priority 0 31 ${_module} thread_priority)
    config_integer(_${_module}_period 1 2147483647 ${_module} period_ticks)
    config_integer(_${_module}_offline 1 2147483647 ${_module} offline_timeout_ms)
endforeach()

