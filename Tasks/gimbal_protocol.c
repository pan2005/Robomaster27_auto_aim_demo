#include "gimbal_protocol.h"

#include <string.h>

enum
{
    PARSER_SYNC_0 = 0,
    PARSER_SYNC_1,
    PARSER_HEADER,
    PARSER_PAYLOAD,
    PARSER_CRC
};

uint16_t GimbalProtocol_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void GimbalProtocolParser_Init(GimbalProtocolParser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

static void ParserRestart(GimbalProtocolParser_t *parser, uint8_t byte)
{
    parser->state = (byte == GIMBAL_PROTOCOL_SYNC_0) ? PARSER_SYNC_1
                                                     : PARSER_SYNC_0;
    parser->header_index = 0U;
    parser->payload_index = 0U;
    parser->crc_index = 0U;
}

int GimbalProtocolParser_Push(GimbalProtocolParser_t *parser, uint8_t byte,
                              GimbalProtocolFrame_t *frame)
{
    switch (parser->state)
    {
        case PARSER_SYNC_0:
            if (byte == GIMBAL_PROTOCOL_SYNC_0) parser->state = PARSER_SYNC_1;
            break;

        case PARSER_SYNC_1:
            if (byte == GIMBAL_PROTOCOL_SYNC_1)
            {
                parser->state = PARSER_HEADER;
                parser->header_index = 0U;
            }
            else
            {
                ParserRestart(parser, byte);
            }
            break;

        case PARSER_HEADER:
            parser->header[parser->header_index++] = byte;
            if (parser->header_index == sizeof(parser->header))
            {
                if (parser->header[0] != GIMBAL_PROTOCOL_VERSION)
                {
                    ++parser->version_errors;
                    ParserRestart(parser, byte);
                }
                else if (parser->header[2] > GIMBAL_PROTOCOL_MAX_PAYLOAD)
                {
                    ++parser->length_errors;
                    ParserRestart(parser, byte);
                }
                else
                {
                    parser->payload_index = 0U;
                    parser->crc_index = 0U;
                    parser->state = (parser->header[2] == 0U)
                                  ? PARSER_CRC : PARSER_PAYLOAD;
                }
            }
            break;

        case PARSER_PAYLOAD:
            parser->payload[parser->payload_index++] = byte;
            if (parser->payload_index == parser->header[2])
            {
                parser->state = PARSER_CRC;
            }
            break;

        case PARSER_CRC:
            parser->crc_bytes[parser->crc_index++] = byte;
            if (parser->crc_index == 2U)
            {
                uint8_t crc_input[5U + GIMBAL_PROTOCOL_MAX_PAYLOAD];
                memcpy(crc_input, parser->header, sizeof(parser->header));
                memcpy(&crc_input[sizeof(parser->header)], parser->payload,
                       parser->header[2]);
                uint16_t expected = GimbalProtocol_Crc16(
                    crc_input, sizeof(parser->header) + parser->header[2]);
                uint16_t received = (uint16_t)parser->crc_bytes[0] |
                                    ((uint16_t)parser->crc_bytes[1] << 8);
                if (expected == received)
                {
                    frame->type = parser->header[1];
                    frame->length = parser->header[2];
                    frame->sequence = (uint16_t)parser->header[3] |
                                      ((uint16_t)parser->header[4] << 8);
                    memcpy(frame->payload, parser->payload, frame->length);
                    ParserRestart(parser, 0U);
                    return 1;
                }
                ++parser->crc_errors;
                ParserRestart(parser, byte);
            }
            break;

        default:
            ParserRestart(parser, byte);
            break;
    }
    return 0;
}

size_t GimbalProtocol_Encode(uint8_t type, uint16_t sequence,
                             const uint8_t *payload, uint8_t payload_length,
                             uint8_t *output, size_t output_size)
{
    size_t frame_length = 9U + payload_length;
    if ((payload_length > GIMBAL_PROTOCOL_MAX_PAYLOAD) ||
        (output_size < frame_length)) return 0U;

    output[0] = GIMBAL_PROTOCOL_SYNC_0;
    output[1] = GIMBAL_PROTOCOL_SYNC_1;
    output[2] = GIMBAL_PROTOCOL_VERSION;
    output[3] = type;
    output[4] = payload_length;
    GimbalProtocol_WriteU16(&output[5], sequence);
    if ((payload_length > 0U) && (payload != NULL))
    {
        memcpy(&output[7], payload, payload_length);
    }
    uint16_t crc = GimbalProtocol_Crc16(&output[2], 5U + payload_length);
    GimbalProtocol_WriteU16(&output[7U + payload_length], crc);
    return frame_length;
}

uint16_t GimbalProtocol_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

int16_t GimbalProtocol_ReadI16(const uint8_t *data)
{
    return (int16_t)GimbalProtocol_ReadU16(data);
}

uint32_t GimbalProtocol_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

int32_t GimbalProtocol_ReadI32(const uint8_t *data)
{
    return (int32_t)GimbalProtocol_ReadU32(data);
}

void GimbalProtocol_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

void GimbalProtocol_WriteI16(uint8_t *data, int16_t value)
{
    GimbalProtocol_WriteU16(data, (uint16_t)value);
}

void GimbalProtocol_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

void GimbalProtocol_WriteI32(uint8_t *data, int32_t value)
{
    GimbalProtocol_WriteU32(data, (uint32_t)value);
}
