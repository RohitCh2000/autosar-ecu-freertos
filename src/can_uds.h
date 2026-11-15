/*
 * can_uds.h - CAN signal packing and UDS (ISO 14229) diagnostics.
 *
 * Personal project based on professional automotive embedded experience.
 * No employer code. AUTOSAR Classic-style: a Com-like signal packing
 * layer plus a Dcm-like diagnostic service handler, in plain C11.
 */
#ifndef CAN_UDS_H
#define CAN_UDS_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* CAN signal layer                                                    */
/*                                                                     */
/* Frame "PowertrainStatus", CAN ID 0x100, 8 bytes, Intel (little-     */
/* endian) byte order:                                                 */
/*                                                                     */
/*   Signal            Start  Len  Factor  Offset  Unit                */
/*   VehicleSpeed        0     16    0.01      0    km/h               */
/*   EngineRpm          16     16    1         0    rpm                */
/*   ThrottlePosition   32      8    1         0    %                  */
/*   CoolantTemp        40      8    1       -40    deg C              */
/*   (unused)           48     16    -         -                       */
/* ------------------------------------------------------------------ */

#define CAN_POWERTRAIN_ID   0x100u
#define CAN_FRAME_LEN       8u

typedef struct {
    double vehicle_speed_kph;   /* 0 .. 250.00 km/h, resolution 0.01 */
    double engine_rpm;          /* 0 .. 8000 rpm,    resolution 1    */
    double throttle_percent;    /* 0 .. 100 %,       resolution 1    */
    double coolant_temp_c;      /* -40 .. 150 deg C, resolution 1    */
} PowertrainSignals;

/* Pack physical signal values into an 8-byte CAN frame. */
void can_pack_powertrain(const PowertrainSignals *in, uint8_t frame[CAN_FRAME_LEN]);

/* Unpack an 8-byte CAN frame back into physical signal values. */
void can_unpack_powertrain(const uint8_t frame[CAN_FRAME_LEN], PowertrainSignals *out);

/* ------------------------------------------------------------------ */
/* UDS (ISO 14229) diagnostic layer                                    */
/* ------------------------------------------------------------------ */

#define UDS_SESSION_DEFAULT   0x01u
#define UDS_SESSION_EXTENDED  0x03u

/* Negative response codes used by this example. */
#define UDS_NRC_SERVICE_NOT_SUPPORTED                 0x11u
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED             0x12u
#define UDS_NRC_INCORRECT_LENGTH_OR_INVALID_FORMAT    0x13u
#define UDS_NRC_REQUEST_OUT_OF_RANGE                  0x31u
#define UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION      0x7Fu

/* Data identifiers for ReadDataByIdentifier (0x22). */
#define UDS_DID_VIN            0xF190u  /* VIN-like string, 17 chars   */
#define UDS_DID_VEHICLE_SPEED  0x0100u  /* uint16 BE, km/h * 100       */
#define UDS_DID_ENGINE_RPM     0x0101u  /* uint16 BE, rpm              */

typedef struct {
    uint8_t  session;            /* UDS_SESSION_DEFAULT / _EXTENDED */
    bool     reset_requested;    /* set by a successful ECUReset    */
    uint16_t vehicle_speed_raw;  /* km/h * 100, served via 0x22     */
    uint16_t engine_rpm_raw;     /* rpm,        served via 0x22     */
    char     vin[18];            /* 17 chars + NUL, served via 0x22 */
} UdsServer;

/* Initialise the server in the default session with demo data that
 * matches the CAN demo values in main.c. */
void uds_init(UdsServer *srv);

/*
 * Handle one UDS request.
 *   req / req_len : request bytes, e.g. { 0x22, 0xF1, 0x90 }
 *   resp          : caller buffer, at least 32 bytes
 *   resp_len      : out, response length in bytes
 * Returns 0 on success (a response, positive or negative, was
 * produced), -1 if the request was empty.
 */
int uds_handle(UdsServer *srv, const uint8_t *req, size_t req_len,
               uint8_t *resp, size_t *resp_len);

#endif /* CAN_UDS_H */
