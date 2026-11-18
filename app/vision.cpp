#include "vision.hpp"

#include "bsp_usart.hpp"
#include "crc.hpp"
#include "main.h"
#include "types.hpp"
#include "user_config.hpp"

#include <cstring>

namespace robot::vision
{
namespace
{
namespace uart = bsp::usart;
namespace cfg = user_config::vision;
using pnxtypes::status;

constexpr auto resource = user_config::vision_uart;
constexpr auto port = resource.instance;
static_assert(port == uart::handle::none || (resource.dma.rx && resource.dma.tx),
    "Vision requires RX and TX DMA");

constexpr uint8_t error_header[]{0xA5, 0x5A, 0x01, 0x01};
constexpr uint32_t error_frame_size = 16;
constexpr uint32_t chunk_size = 32;
constexpr UINT queue_depth = 8;
constexpr UINT metadata_words = 3;
static_assert(chunk_size % sizeof(ULONG) == 0);
constexpr UINT message_words = metadata_words + chunk_size / sizeof(ULONG);
static_assert(message_words <= TX_16_ULONG);

memory::buffer<chunk_size> dma_rx RAM_D1_BSS{};
TX_QUEUE rx_queue{};
CHAR rx_queue_name[] = "vision_rx";
ULONG rx_storage[queue_depth * message_words]{};
ULONG chunk_sequence = 0;

// IRQ 中只拷贝字节和时间，DMA 缓冲由 BSP 继续使用。
void on_rx(uart::handle, uint8_t* data, uint32_t size)
{
    ULONG message[message_words]{size, HAL_GetTick(), chunk_sequence++};
    std::memcpy(message + metadata_words, data, size);
    (void)tx_queue_send(&rx_queue, message, TX_NO_WAIT);
}

uint16_t read_u16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0] | (uint16_t(data[1]) << 8U));
}

int32_t read_i16(const uint8_t* data)
{
    const auto bits = read_u16(data);
    return int32_t(bits) - (bits >= 0x8000U ? 65536 : 0);
}

bool decode_error(const uint8_t* frame, uint32_t received_ms, Error& error)
{
    const auto flags = frame[6];
    if (std::memcmp(frame, error_header, sizeof(error_header)) != 0 || frame[7] != 0 ||
        (flags & ~7U) != 0 || ((flags & 4U) != 0 && (flags & 2U) == 0) ||
        !crc::verify_crc16_checksum(frame, error_frame_size))
    {
        return false;
    }

    const auto ex_mm = read_i16(frame + 8);
    const auto ey_mm = read_i16(frame + 10);
    const auto eyaw_mrad = read_i16(frame + 12);
    const bool valid = (flags & 1U) != 0 && ex_mm >= -3000 && ex_mm <= 3000 &&
        ey_mm >= -3000 && ey_mm <= 3000 && eyaw_mrad >= -3142 && eyaw_mrad <= 3142;
    error = {ex_mm * 0.001f, ey_mm * 0.001f, eyaw_mrad * 0.001f, read_u16(frame + 4),
        received_ms, valid};
    return true;
}

void receive(Error& error)
{
    static uint8_t frame[error_frame_size]{};
    static uint32_t used = 0;
    static ULONG next_chunk = 0;
    static uint32_t previous_ms = 0;

    if (uart::rx_status(port) != status::ok)
    {
        used = 0;
        error.valid = false;
        (void)tx_queue_flush(&rx_queue);
        (void)uart::restart_rx(port);
        return;
    }

    // 每轮限量处理，留出时间给同一线程的控制逻辑。
    for (UINT i = 0; i < queue_depth; ++i)
    {
        ULONG message[message_words];
        if (tx_queue_receive(&rx_queue, message, TX_NO_WAIT) != TX_SUCCESS)
        {
            break;
        }

        const uint32_t received_ms = message[1];
        if (message[2] != next_chunk ||
            uint32_t(received_ms - previous_ms) > cfg::position_timeout_ms)
        {
            used = 0;
        }
        next_chunk = message[2] + 1U;
        previous_ms = received_ms;

        const auto* bytes = reinterpret_cast<const uint8_t*>(message + metadata_words);
        for (ULONG j = 0; j < message[0]; ++j)
        {
            frame[used++] = bytes[j];
            if (used != error_frame_size)
            {
                continue;
            }
            if (decode_error(frame, received_ms, error))
            {
                used = 0;
            }
            else
            {
                // 错帧滑过一个字节，保留后面可能的帧头。
                used = error_frame_size - 1U;
                std::memmove(frame, frame + 1, used);
            }
        }
    }
}

void send_attitude(const imu::attitude::state& attitude, uint32_t now)
{
    static_assert(sizeof(float) == 4 && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

    const bool running = attitude.online && attitude.stage == imu::attitude::phase::running &&
                         attitude.result == status::ok;
    uint8_t frame[30]{0xA5, 0x5A, 0x01, 0x03};
    frame[6] = fresh(running, attitude.received_ms, now, cfg::imu_timeout_ms) ? 1U : 0U;

    const uint16_t sequence = static_cast<uint16_t>(attitude.sequence);
    const auto& q = attitude.quaternion;
    const float wxyz[]{q.w, q.x, q.y, q.z};
    std::memcpy(frame + 4, &sequence, sizeof(sequence));
    std::memcpy(frame + 8, &attitude.received_ms, sizeof(attitude.received_ms));
    std::memcpy(frame + 12, wxyz, sizeof(wxyz));
    crc::append_crc16_checksum(frame, sizeof(frame));

    // BSP 拷贝到自己的 DMA 缓冲，下一周期再尝试发送最新姿态。
    (void)uart::transmit(port, frame, sizeof(frame), 0);
}
} // namespace

pnxtypes::status init()
{
    if constexpr (port == uart::handle::none)
    {
        return status::ok;
    }

    uart::line_config line{};
    line.baud_rate = cfg::baud;
    auto result = uart::configure(port, line);
    if (result != status::ok)
    {
        return result;
    }

    result = uart::init(resource, uart::mode::dma);
    if (result != status::ok)
    {
        return result;
    }

    if (tx_queue_create(&rx_queue, rx_queue_name, message_words, rx_storage, sizeof(rx_storage)) !=
        TX_SUCCESS)
    {
        return status::error;
    }

    result = uart::start_rx_to_idle(port, dma_rx, uart::rx_callback::bind<&on_rx>());
    if (result != status::ok && uart::stop_rx(port) == status::ok)
    {
        (void)tx_queue_delete(&rx_queue);
    }
    return result;
}

void update(Error& error, const imu::attitude::state& attitude)
{
    if constexpr (port == uart::handle::none)
    {
        return;
    }

    receive(error);
    const uint32_t now = HAL_GetTick();
    error.valid = fresh(error.valid, error.received_ms, now, cfg::position_timeout_ms);

    static uint32_t last_tx = now - cfg::imu_period_ms;
    if (uint32_t(now - last_tx) >= cfg::imu_period_ms)
    {
        last_tx = now;
        send_attitude(attitude, now);
    }
}
} // namespace robot::vision