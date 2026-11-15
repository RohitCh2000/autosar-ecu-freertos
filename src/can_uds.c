/*
 * can_uds.c - Implementation of the CAN signal layer and the
 * UDS (ISO 14229) handler declared in can_uds.h.
 */
#include "can_uds.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Generic Intel (little-endian) bit-field helpers                     */
/* ------------------------------------------------------------------ */

static void insert_bits(uint8_t frame[CAN_FRAME_LEN], unsigned start_bit,
                        unsigned length, uint64_t raw)
{
    for (unsigned i = 0; i < length; i++) {
        unsigned bit = start_bit + i;
        if ((raw >> i) & 1u) {
            frame[bit / 8] |= (uint8_t)(1u << (bit % 8));
        } else {
            frame[bit / 8] &= (uint8_t)~(1u << (bit % 8));
        }
    }
}

static uint64_t extract_bits(const uint8_t frame[CAN_FRAME_LEN],
                             unsigned start_bit, unsigned length)
{
    uint64_t raw = 0;
    for (unsigned i = 0; i < length; i++) {
        unsigned bit = start_bit + i;
        if ((frame[bit / 8] >> (bit % 8)) & 1u) {
            raw |= (1ull << i);
        }
    }
    return raw;
}

static double clamp(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ------------------------------------------------------------------ */
/* CAN packing / unpacking                                             */
/* ------------------------------------------------------------------ */

void can_pack_powertrain(const PowertrainSignals *in, uint8_t frame[CAN_FRAME_LEN])
{
    memset(frame, 0, CAN_FRAME_LEN);

    /* physical = raw * factor + offset  =>  raw = (physical - offset) / factor */
    uint64_t speed_raw    = (uint64_t)(clamp(in->vehicle_speed_kph, 0.0, 250.0) / 0.01 + 0.5);
    uint64_t rpm_raw      = (uint64_t)(clamp(in->engine_rpm, 0.0, 8000.0) + 0.5);
    uint64_t throttle_raw = (uint64_t)(clamp(in->throttle_percent, 0.0, 100.0) + 0.5);
    uint64_t coolant_raw  = (uint64_t)(clamp(in->coolant_temp_c, -40.0, 150.0) + 40.0 + 0.5);

    insert_bits(frame, 0,  16, speed_raw);
    insert_bits(frame, 16, 16, rpm_raw);
    insert_bits(frame, 32, 8,  throttle_raw);
    insert_bits(frame, 40, 8,  coolant_raw);
}

void can_unpack_powertrain(const uint8_t frame[CAN_FRAME_LEN], PowertrainSignals *out)
{
    out->vehicle_speed_kph = (double)extract_bits(frame, 0, 16) * 0.01;
    out->engine_rpm        = (double)extract_bits(frame, 16, 16) * 1.0;
    out->throttle_percent  = (double)extract_bits(frame, 32, 8) * 1.0;
    out->coolant_temp_c    = (double)extract_bits(frame, 40, 8) * 1.0 - 40.0;
}

/* ------------------------------------------------------------------ */
/* UDS helpers                                                         */
/* ------------------------------------------------------------------ */

static size_t make_nrc(uint8_t *resp, uint8_t sid, uint8_t nrc)
{
    resp[0] = 0x7F;
    resp[1] = sid;
    resp[2] = nrc;
    return 3;
}

void uds_init(UdsServer *srv)
{
    srv->session           = UDS_SESSION_DEFAULT;
    srv->reset_requested   = false;
    srv->vehicle_speed_raw = 8750;  /* 87.50 km/h */
    srv->engine_rpm_raw    = 3200;  /* 3200 rpm   */
    memcpy(srv->vin, "1HGCM82633A004352", 18); /* fictional VIN-like string */
}

/* Stored diagnostic trouble codes for this example: exactly one. */
static const struct { uint32_t dtc; uint8_t status; } kStoredDtcs[] = {
    { 0x012345u, 0x2Fu },  /* DTC 0x012345, statusOfDTC 0x2F */
};

/* ------------------------------------------------------------------ */
/* UDS service handlers                                                */
/* ------------------------------------------------------------------ */

static size_t handle_session_control(UdsServer *srv, const uint8_t *req,
                                     size_t req_len, uint8_t *resp)
{
    if (req_len != 2) {
        return make_nrc(resp, req[0], UDS_NRC_INCORRECT_LENGTH_OR_INVALID_FORMAT);
    }
    uint8_t sub = req[1] & 0x7Fu; /* bit 7 = suppress-response flag */
    if (sub != UDS_SESSION_DEFAULT && sub != UDS_SESSION_EXTENDED) {
        return make_nrc(resp, req[0], UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    }
    srv->session = sub;
    /* Positive response: 0x50, sub-function, P2 (0x0032 = 50 ms),
     * P2* (0x01F4 = 5000 ms), both big-endian per ISO 14229. */
    resp[0] = 0x50; resp[1] = sub;
    resp[2] = 0x00; resp[3] = 0x32;
    resp[4] = 0x01; resp[5] = 0xF4;
    return 6;
}

static size_t handle_ecu_reset(UdsServer *srv, const uint8_t *req,
                               size_t req_len, uint8_t *resp)
{
    if (req_len != 2) {
        return make_nrc(resp, req[0], UDS_NRC_INCORRECT_LENGTH_OR_INVALID_FORMAT);
    }
    if (req[1] != 0x01u) { /* only hardReset (0x01) is implemented */
        return make_nrc(resp, req[0], UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    }
    srv->reset_requested = true;
    srv->session = UDS_SESSION_DEFAULT; /* a reset returns to default */
    resp[0] = 0x51; resp[1] = 0x01;
    return 2;
}

static size_t handle_read_data_by_id(UdsServer *srv, const uint8_t *req,
                                     size_t req_len, uint8_t *resp)
{
    if (req_len != 3) {
        return make_nrc(resp, req[0], UDS_NRC_INCORRECT_LENGTH_OR_INVALID_FORMAT);
    }
    uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
    resp[0] = 0x62; resp[1] = req[1]; resp[2] = req[2];

    switch (did) {
    case UDS_DID_VIN:
        memcpy(&resp[3], srv->vin, 17);
        return 3 + 17;
    case UDS_DID_VEHICLE_SPEED:
        resp[3] = (uint8_t)(srv->vehicle_speed_raw >> 8);
        resp[4] = (uint8_t)(srv->vehicle_speed_raw & 0xFFu);
        return 5;
    case UDS_DID_ENGINE_RPM:
        resp[3] = (uint8_t)(srv->engine_rpm_raw >> 8);
        resp[4] = (uint8_t)(srv->engine_rpm_raw & 0xFFu);
        return 5;
    default:
        return make_nrc(resp, req[0], UDS_NRC_REQUEST_OUT_OF_RANGE);
    }
}

static size_t handle_read_dtc_info(UdsServer *srv, const uint8_t *req,
                                   size_t req_len, uint8_t *resp)
{
    /* This example only serves reportDTCByStatusMask (0x02), and
     * only in the extended session - a common OEM restriction. */
    if (srv->session != UDS_SESSION_EXTENDED) {
        return make_nrc(resp, req[0], UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);
    }
    if (req_len != 3) {
        return make_nrc(resp, req[0], UDS_NRC_INCORRECT_LENGTH_OR_INVALID_FORMAT);
    }
    if (req[1] != 0x02u) {
        return make_nrc(resp, req[0], UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    }
    uint8_t status_mask = req[2];
    resp[0] = 0x59; resp[1] = 0x02; resp[2] = 0xFF; /* DTCStatusAvailabilityMask */
    size_t n = 3;
    for (size_t i = 0; i < sizeof(kStoredDtcs) / sizeof(kStoredDtcs[0]); i++) {
        if ((kStoredDtcs[i].status & status_mask) != 0) {
            resp[n++] = (uint8_t)(kStoredDtcs[i].dtc >> 16);
            resp[n++] = (uint8_t)(kStoredDtcs[i].dtc >> 8);
            resp[n++] = (uint8_t)(kStoredDtcs[i].dtc & 0xFFu);
            resp[n++] = kStoredDtcs[i].status;
        }
    }
    return n;
}

int uds_handle(UdsServer *srv, const uint8_t *req, size_t req_len,
               uint8_t *resp, size_t *resp_len)
{
    if (req == NULL || req_len == 0) {
        return -1;
    }
    size_t n;
    switch (req[0]) {
    case 0x10: n = handle_session_control(srv, req, req_len, resp); break;
    case 0x11: n = handle_ecu_reset(srv, req, req_len, resp);       break;
    case 0x22: n = handle_read_data_by_id(srv, req, req_len, resp); break;
    case 0x19: n = handle_read_dtc_info(srv, req, req_len, resp);   break;
    default:   n = make_nrc(resp, req[0], UDS_NRC_SERVICE_NOT_SUPPORTED); break;
    }
    *resp_len = n;
    return 0;
}
