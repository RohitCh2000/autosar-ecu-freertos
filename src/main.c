/*
 * main.c - Demo for autosar-ecu-freertos.
 *
 * 1. Packs a powertrain CAN frame, prints it, unpacks it and checks
 *    the round-trip.
 * 2. Runs a deterministic UDS (ISO 14229) request sequence against
 *    the handler in can_uds.c, printing every request/response as
 *    hex with a plain-English interpretation, including a negative
 *    response for reading DTCs in the wrong session and one for an
 *    unsupported service.
 *
 * Personal project; no employer code.
 */
#include <stdio.h>
#include <string.h>

#include "can_uds.h"

static void print_hex(const char *label, const uint8_t *data, size_t len)
{
    printf("%s", label);
    for (size_t i = 0; i < len; i++) {
        printf(" %02X", data[i]);
    }
    printf("\n");
}

static void uds_step(UdsServer *srv, const char *title,
                     const uint8_t *req, size_t req_len, const char *expect)
{
    uint8_t resp[64];
    size_t resp_len = 0;
    printf("-- %s\n", title);
    print_hex("   Req :", req, req_len);
    if (uds_handle(srv, req, req_len, resp, &resp_len) == 0) {
        print_hex("   Resp:", resp, resp_len);
    }
    printf("   -> %s\n", expect);
}

int main(void)
{
    printf("=== AUTOSAR Classic-style ECU demo: CAN + UDS ===\n\n");

    /* ---------------- CAN signal pack / unpack ---------------- */
    printf("--- CAN: PowertrainStatus (ID 0x100, 8 bytes, Intel) ---\n");
    PowertrainSignals in = {
        .vehicle_speed_kph = 87.50,
        .engine_rpm        = 3200.0,
        .throttle_percent  = 42.0,
        .coolant_temp_c    = 92.0,
    };
    printf("Input signals: speed=%.2f km/h, rpm=%.0f, throttle=%.0f %%, coolant=%.0f C\n",
           in.vehicle_speed_kph, in.engine_rpm, in.throttle_percent, in.coolant_temp_c);

    uint8_t frame[CAN_FRAME_LEN];
    can_pack_powertrain(&in, frame);
    print_hex("Packed CAN frame:", frame, CAN_FRAME_LEN);

    PowertrainSignals out;
    can_unpack_powertrain(frame, &out);
    printf("Unpacked signals: speed=%.2f km/h, rpm=%.0f, throttle=%.0f %%, coolant=%.0f C\n",
           out.vehicle_speed_kph, out.engine_rpm, out.throttle_percent, out.coolant_temp_c);

    int roundtrip_ok =
        out.vehicle_speed_kph == in.vehicle_speed_kph &&
        out.engine_rpm        == in.engine_rpm &&
        out.throttle_percent  == in.throttle_percent &&
        out.coolant_temp_c    == in.coolant_temp_c;
    printf("Round-trip check: %s\n\n", roundtrip_ok ? "PASS (decoded values match inputs)" :
                                                     "FAIL");

    /* ---------------- UDS diagnostic sequence ----------------- */
    printf("--- UDS (ISO 14229) diagnostic session ---\n");
    UdsServer srv;
    uds_init(&srv);
    printf("ECU starts in the DEFAULT session.\n");

    static const uint8_t req_vin[]      = { 0x22, 0xF1, 0x90 };
    static const uint8_t req_dtc[]      = { 0x19, 0x02, 0xFF };
    static const uint8_t req_ext[]      = { 0x10, 0x03 };
    static const uint8_t req_speed[]    = { 0x22, 0x01, 0x00 };
    static const uint8_t req_write[]    = { 0x2E, 0x01, 0x00, 0x00 };
    static const uint8_t req_reset[]    = { 0x11, 0x01 };

    uds_step(&srv, "1. ReadDataByIdentifier - VIN (DID 0xF190)",
             req_vin, sizeof(req_vin),
             "Positive response 0x62: VIN = \"1HGCM82633A004352\" (17 chars after the DID).");

    uds_step(&srv, "2. ReadDTCInformation in the DEFAULT session (wrong session)",
             req_dtc, sizeof(req_dtc),
             "Negative response 0x7F 0x19 0x7F: service not supported in the active session.");

    uds_step(&srv, "3. DiagnosticSessionControl - enter EXTENDED session (0x03)",
             req_ext, sizeof(req_ext),
             "Positive response 0x50: extended session, P2 = 50 ms (0x0032), P2* = 5000 ms (0x01F4).");

    uds_step(&srv, "4. ReadDTCInformation - report DTCs by status mask 0xFF",
             req_dtc, sizeof(req_dtc),
             "Positive response 0x59: 1 stored DTC, 0x012345, statusOfDTC = 0x2F.");

    uds_step(&srv, "5. ReadDataByIdentifier - vehicle speed (DID 0x0100)",
             req_speed, sizeof(req_speed),
             "Positive response 0x62: raw 0x222E = 8750 -> 87.50 km/h (factor 0.01).");

    uds_step(&srv, "6. WriteDataByIdentifier (0x2E) - not implemented",
             req_write, sizeof(req_write),
             "Negative response 0x7F 0x2E 0x11: service not supported.");

    uds_step(&srv, "7. ECUReset - hard reset (0x01)",
             req_reset, sizeof(req_reset),
             "Positive response 0x51: reset requested, ECU returns to the DEFAULT session.");

    printf("\nDemo complete. Final session = 0x%02X (default), reset_requested = %s\n",
           srv.session, srv.reset_requested ? "yes" : "no");
    return roundtrip_ok ? 0 : 1;
}
