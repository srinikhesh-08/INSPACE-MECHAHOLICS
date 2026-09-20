#ifndef GNSS_NEO_M9N_H
#define GNSS_NEO_M9N_H

#include "main.h"      // CubeMX main.h pulls in the right stm32xxxx_hal.h for your family
#include <cstdint>

// Everything the NEO-M9N can give you that is relevant for the telemetry packet.
struct GnssData {
    // UTC date/time from the receiver  -> gnss_time
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    bool     time_valid;

    // Fix status
    uint8_t  fix_type;          // 0 none, 2 = 2D, 3 = 3D, 4 = GNSS+DR, 5 = time only
    bool     fix_ok;            // gnssFixOK flag from the receiver
    uint8_t  num_sats;          // -> gnss_sats

    // Position
    double   latitude_deg;      // -> latitude
    double   longitude_deg;     // -> longitude
    float    altitude_msl_m;    // -> gnss_altitude (above mean sea level)
    float    altitude_ellipsoid_m;

    // Extras that are useful for the FSM (free in the same UBX message)
    float    ground_speed_mps;
    float    vel_down_mps;      // positive = descending (NED frame)
    float    h_acc_m;
    float    v_acc_m;

    uint32_t last_update_tick_ms;   // HAL_GetTick() when this solution was received
};

class GnssNeoM9N {
public:
    // huart must be configured in CubeMX with:
    //   - baud = 38400 (NEO-M9N factory default), 8N1
    //   - USARTx_RX DMA request in CIRCULAR mode, byte width
    explicit GnssNeoM9N(UART_HandleTypeDef* huart) : huart_(huart) {}

    // configure = true: sends UBX-CFG-VALSET to (a) disable NMEA on UART1,
    // (b) enable UBX-NAV-PVT, (c) set 10 Hz, (d) set airborne <4g dynamic model.
    // Then starts circular DMA reception.
    bool begin(bool configure = true);

    // Call this often from the main loop (at least every ~50 ms). Non-blocking.
    void poll();

    // True if a new NAV-PVT solution arrived since the last getData().
    bool hasNewData() const { return new_data_; }

    // Copies the latest solution to 'out' and clears the new-data flag.
    // Returns false if no solution has ever been received.
    bool getData(GnssData& out);

    // Convenience: 3D fix and >= 4 satellites
    bool hasFix() const { return data_.fix_ok && data_.fix_type >= 3; }

private:
    static constexpr uint16_t RX_BUF_SIZE     = 512;
    static constexpr uint16_t MAX_PAYLOAD     = 100;
    static constexpr uint16_t NAV_PVT_LEN     = 92;

    enum class ParseState : uint8_t {
        SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
    };

    void feed(uint8_t b);
    void handleMessage();
    void parseNavPvt(const uint8_t* p);
    bool sendValset(uint32_t key, const uint8_t* value, uint8_t value_len);

    UART_HandleTypeDef* huart_;
    uint8_t  rx_buf_[RX_BUF_SIZE] = {0};
    uint16_t tail_ = 0;

    ParseState state_ = ParseState::SYNC1;
    uint8_t  cls_ = 0, id_ = 0, ck_a_ = 0, ck_b_ = 0;
    uint16_t len_ = 0, idx_ = 0;
    uint8_t  payload_[MAX_PAYLOAD] = {0};

    GnssData data_ = {};
    bool     new_data_ = false;
    bool     have_data_ = false;
};

#endif // GNSS_NEO_M9N_H
