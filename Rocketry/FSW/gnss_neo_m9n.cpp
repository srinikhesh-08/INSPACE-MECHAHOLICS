#include "gnss_neo_m9n.h"
#include <cstring>

// ---- UBX configuration key IDs (u-blox M9 generation, interface description) ----
static constexpr uint32_t CFG_UART1OUTPROT_NMEA   = 0x10740002; // L  (1 byte)
static constexpr uint32_t CFG_MSGOUT_NAV_PVT_UART1 = 0x20910007; // U1 (1 byte)
static constexpr uint32_t CFG_NAVSPG_DYNMODEL     = 0x20110021; // E1 (1 byte), 8 = airborne <4g
static constexpr uint32_t CFG_RATE_MEAS           = 0x30210001; // U2 (2 bytes, ms)

template <typename T>
static inline T rd(const uint8_t* p, size_t off) {
    T v;
    memcpy(&v, p + off, sizeof(T));   // STM32 is little-endian, same as UBX
    return v;
}

// ------------------------------------------------------------------------------
bool GnssNeoM9N::begin(bool configure)
{
    if (configure) {
        const uint8_t nmea_off = 0;
        const uint8_t pvt_on   = 1;
        const uint8_t dyn_air  = 8;                 // airborne <4g
        const uint16_t meas_ms = 100;               // 10 Hz
        const uint8_t rate[2]  = { (uint8_t)(meas_ms & 0xFF), (uint8_t)(meas_ms >> 8) };

        sendValset(CFG_UART1OUTPROT_NMEA,    &nmea_off, 1);
        sendValset(CFG_MSGOUT_NAV_PVT_UART1, &pvt_on,   1);
        sendValset(CFG_NAVSPG_DYNMODEL,      &dyn_air,  1);
        sendValset(CFG_RATE_MEAS,            rate,      2);
    }

    tail_ = 0;
    state_ = ParseState::SYNC1;
    return HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUF_SIZE) == HAL_OK;
}

// ------------------------------------------------------------------------------
void GnssNeoM9N::poll()
{
    // Restart DMA if a UART error (e.g. overrun) stopped reception
    if (huart_->RxState != HAL_UART_STATE_BUSY_RX) {
        tail_ = 0;
        HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUF_SIZE);
        return;
    }

    const uint16_t head = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    while (tail_ != head) {
        feed(rx_buf_[tail_]);
        tail_ = (tail_ + 1) % RX_BUF_SIZE;
    }
}

// ------------------------------------------------------------------------------
bool GnssNeoM9N::getData(GnssData& out)
{
    if (!have_data_) return false;
    out = data_;
    new_data_ = false;
    return true;
}

// ------------------------------------------------------------------------------
// UBX frame parser: B5 62 | class | id | len(LE16) | payload | CK_A CK_B
void GnssNeoM9N::feed(uint8_t b)
{
    switch (state_) {
    case ParseState::SYNC1:
        if (b == 0xB5) state_ = ParseState::SYNC2;
        break;

    case ParseState::SYNC2:
        if (b == 0x62)      state_ = ParseState::CLASS;
        else if (b != 0xB5) state_ = ParseState::SYNC1;
        break;

    case ParseState::CLASS:
        cls_ = b; ck_a_ = 0; ck_b_ = 0;
        ck_a_ += b; ck_b_ += ck_a_;
        state_ = ParseState::ID;
        break;

    case ParseState::ID:
        id_ = b;
        ck_a_ += b; ck_b_ += ck_a_;
        state_ = ParseState::LEN1;
        break;

    case ParseState::LEN1:
        len_ = b;
        ck_a_ += b; ck_b_ += ck_a_;
        state_ = ParseState::LEN2;
        break;

    case ParseState::LEN2:
        len_ |= (uint16_t)b << 8;
        ck_a_ += b; ck_b_ += ck_a_;
        idx_ = 0;
        if (len_ > MAX_PAYLOAD)  state_ = ParseState::SYNC1;   // not something we care about
        else if (len_ == 0)      state_ = ParseState::CK_A;
        else                     state_ = ParseState::PAYLOAD;
        break;

    case ParseState::PAYLOAD:
        payload_[idx_++] = b;
        ck_a_ += b; ck_b_ += ck_a_;
        if (idx_ >= len_) state_ = ParseState::CK_A;
        break;

    case ParseState::CK_A:
        state_ = (b == ck_a_) ? ParseState::CK_B : ParseState::SYNC1;
        break;

    case ParseState::CK_B:
        if (b == ck_b_) handleMessage();
        state_ = ParseState::SYNC1;
        break;
    }
}

// ------------------------------------------------------------------------------
void GnssNeoM9N::handleMessage()
{
    if (cls_ == 0x01 && id_ == 0x07 && len_ == NAV_PVT_LEN) {   // UBX-NAV-PVT
        parseNavPvt(payload_);
    }
    // UBX-ACK-ACK / ACK-NAK (class 0x05) are ignored here
}

// ------------------------------------------------------------------------------
void GnssNeoM9N::parseNavPvt(const uint8_t* p)
{
    data_.year   = rd<uint16_t>(p, 4);
    data_.month  = p[6];
    data_.day    = p[7];
    data_.hour   = p[8];
    data_.minute = p[9];
    data_.second = p[10];
    const uint8_t valid = p[11];
    data_.time_valid = (valid & 0x02) && (valid & 0x04);   // validTime && fullyResolved

    data_.fix_type = p[20];
    data_.fix_ok   = (p[21] & 0x01) != 0;
    data_.num_sats = p[23];

    data_.longitude_deg        = rd<int32_t>(p, 24) * 1e-7;
    data_.latitude_deg         = rd<int32_t>(p, 28) * 1e-7;
    data_.altitude_ellipsoid_m = rd<int32_t>(p, 32) * 1e-3f;
    data_.altitude_msl_m       = rd<int32_t>(p, 36) * 1e-3f;
    data_.h_acc_m              = rd<uint32_t>(p, 40) * 1e-3f;
    data_.v_acc_m              = rd<uint32_t>(p, 44) * 1e-3f;
    data_.vel_down_mps         = rd<int32_t>(p, 56) * 1e-3f;
    data_.ground_speed_mps     = rd<int32_t>(p, 60) * 1e-3f;

    data_.last_update_tick_ms = HAL_GetTick();
    new_data_  = true;
    have_data_ = true;
}

// ------------------------------------------------------------------------------
// UBX-CFG-VALSET (class 0x06, id 0x8A), RAM layer only
bool GnssNeoM9N::sendValset(uint32_t key, const uint8_t* value, uint8_t value_len)
{
    uint8_t frame[32];
    const uint16_t payload_len = 4 + 4 + value_len;   // version,layers,reserved[2] + key + value
    uint16_t n = 0;

    frame[n++] = 0xB5;
    frame[n++] = 0x62;
    frame[n++] = 0x06;
    frame[n++] = 0x8A;
    frame[n++] = payload_len & 0xFF;
    frame[n++] = payload_len >> 8;

    frame[n++] = 0x00;                // version
    frame[n++] = 0x01;                // layers: RAM
    frame[n++] = 0x00;                // reserved
    frame[n++] = 0x00;                // reserved
    frame[n++] = (uint8_t)(key);
    frame[n++] = (uint8_t)(key >> 8);
    frame[n++] = (uint8_t)(key >> 16);
    frame[n++] = (uint8_t)(key >> 24);
    for (uint8_t i = 0; i < value_len; i++) frame[n++] = value[i];

    uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 2; i < n; i++) { ck_a += frame[i]; ck_b += ck_a; }
    frame[n++] = ck_a;
    frame[n++] = ck_b;

    const bool ok = HAL_UART_Transmit(huart_, frame, n, 100) == HAL_OK;
    HAL_Delay(20);
    return ok;
}
