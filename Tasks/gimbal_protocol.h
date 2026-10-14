#ifndef GIMBAL_PROTOCOL_H
#define GIMBAL_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define GIMBAL_PROTOCOL_SYNC_0          0xA5U
#define GIMBAL_PROTOCOL_SYNC_1          0x5AU
#define GIMBAL_PROTOCOL_VERSION         1U
#define GIMBAL_PROTOCOL_MAX_PAYLOAD     48U
#define GIMBAL_PROTOCOL_MAX_FRAME       (9U + GIMBAL_PROTOCOL_MAX_PAYLOAD)

#define GIMBAL_MSG_HELLO                0x01U
#define GIMBAL_MSG_TIME_SYNC_REQ        0x02U
#define GIMBAL_MSG_CONTROL              0x10U
#define GIMBAL_MSG_DISARM               0x11U
#define GIMBAL_MSG_ESTOP                0x12U
#define GIMBAL_MSG_AIM_TARGET           0x13U
#define GIMBAL_MSG_STATUS               0x80U
#define GIMBAL_MSG_TIME_SYNC_RESP       0x82U
#define GIMBAL_MSG_ATTITUDE             0x81U

#define GIMBAL_CONTROL_ARMED            (1U << 0)
#define GIMBAL_CONTROL_FIRE             (1U << 1)

#define GIMBAL_AIM_VALID                (1U << 0)
#define GIMBAL_AIM_ARMED                (1U << 1)
#define GIMBAL_AIM_FIRE                 (1U << 2)

#define GIMBAL_STATUS_IMU_VALID         (1U << 0)
#define GIMBAL_STATUS_PITCH_ONLINE      (1U << 1)
#define GIMBAL_STATUS_YAW_ONLINE        (1U << 2)
#define GIMBAL_STATUS_REFERENCES_READY  (1U << 3)
#define GIMBAL_STATUS_CONTROL_ENABLED   (1U << 4)
#define GIMBAL_STATUS_LINK_ALIVE        (1U << 5)
#define GIMBAL_STATUS_FIRE_REQUESTED    (1U << 6)
#define GIMBAL_STATUS_SAFE              (1U << 7)
#define GIMBAL_STATUS_IMU_WARMUP_TIMEOUT (1U << 8)

typedef struct
{
    uint8_t type;
    uint8_t length;
    uint16_t sequence;
    uint8_t payload[GIMBAL_PROTOCOL_MAX_PAYLOAD];
} GimbalProtocolFrame_t;

typedef struct
{
    uint8_t state;
    uint8_t header[5];
    uint8_t header_index;
    uint8_t payload[GIMBAL_PROTOCOL_MAX_PAYLOAD];
    uint8_t payload_index;
    uint8_t crc_bytes[2];
    uint8_t crc_index;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
} GimbalProtocolParser_t;

void GimbalProtocolParser_Init(GimbalProtocolParser_t *parser);
int GimbalProtocolParser_Push(GimbalProtocolParser_t *parser, uint8_t byte,
                              GimbalProtocolFrame_t *frame);
uint16_t GimbalProtocol_Crc16(const uint8_t *data, size_t length);
size_t GimbalProtocol_Encode(uint8_t type, uint16_t sequence,
                             const uint8_t *payload, uint8_t payload_length,
                             uint8_t *output, size_t output_size);
uint16_t GimbalProtocol_ReadU16(const uint8_t *data);
int16_t GimbalProtocol_ReadI16(const uint8_t *data);
uint32_t GimbalProtocol_ReadU32(const uint8_t *data);
int32_t GimbalProtocol_ReadI32(const uint8_t *data);
void GimbalProtocol_WriteU16(uint8_t *data, uint16_t value);
void GimbalProtocol_WriteI16(uint8_t *data, int16_t value);
void GimbalProtocol_WriteU32(uint8_t *data, uint32_t value);
void GimbalProtocol_WriteI32(uint8_t *data, int32_t value);

#endif /* GIMBAL_PROTOCOL_H */
