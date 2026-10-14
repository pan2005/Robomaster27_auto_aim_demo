import pathlib
import random
import struct
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protocol import (  # noqa: E402
    CONTROL_ARMED,
    CONTROL_FIRE,
    FrameParser,
    MSG_CONTROL,
    MSG_ATTITUDE,
    MSG_TIME_SYNC_RESP,
    MSG_STATUS,
    VERSION,
    COMMAND_TIMEOUT_MS,
    crc16_ccitt,
    decode_status,
    decode_attitude,
    decode_time_sync_response,
    encode_aim_target,
    encode_control,
    encode_frame,
    sequence_is_newer,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_golden_vector(self):
        body = bytes.fromhex("01 10 05 34 12 03 64 00 9C FF")
        self.assertEqual(crc16_ccitt(body), 0xA234)
        frame = encode_control(0x1234, True, True, 100, -100)
        self.assertEqual(frame.hex(" ").upper(),
                         "A5 5A 01 10 05 34 12 03 64 00 9C FF 34 A2")

    def test_fragmented_and_concatenated(self):
        expected = [encode_frame(MSG_CONTROL, number, bytes([number])) for number in range(5)]
        stream = b"noise" + b"".join(expected)
        parser = FrameParser()
        frames = []
        for byte in stream:
            frames.extend(parser.feed(bytes([byte])))
        self.assertEqual([frame.sequence for frame in frames], list(range(5)))

    def test_bad_crc_recovers(self):
        bad = bytearray(encode_frame(MSG_CONTROL, 1, b"abc"))
        bad[-1] ^= 0x80
        good = encode_frame(MSG_CONTROL, 2, b"xyz")
        parser = FrameParser()
        frames = parser.feed(b"\x00\xA5" + bad + good)
        self.assertEqual([frame.sequence for frame in frames], [2])
        self.assertEqual(parser.crc_errors, 1)

    def test_oversize_noise_recovers(self):
        malformed = b"\xA5\x5A" + struct.pack("<BBBH", VERSION, MSG_CONTROL, 255, 8)
        parser = FrameParser()
        frames = parser.feed(malformed + encode_frame(MSG_CONTROL, 9))
        self.assertEqual(frames[0].sequence, 9)
        self.assertEqual(parser.length_errors, 1)

    def test_status_decode(self):
        payload = struct.pack("<HiiiihhBBHHB", 0xA5, 1000, -2000, 3000, -4000,
                              5, -6, 40, 41, 2, 99, VERSION)
        frame = FrameParser().feed(encode_frame(MSG_STATUS, 7, payload))[0]
        status = decode_status(frame)
        self.assertEqual(status.yaw_target_mdeg, -4000)
        self.assertEqual(status.last_control_sequence, 99)

    def test_aim_target_fixed_point_layout(self):
        frame = FrameParser().feed(encode_aim_target(
            10, True, True, False, 1234, -90000, 45000,
            -1000, 2000, -3000, 4000))[0]
        self.assertEqual(frame.message_type, 0x13)
        self.assertEqual(len(frame.payload), 30)
        self.assertEqual(struct.unpack("<BBIiiiiii", frame.payload),
                         (3, 1, 1234, -90000, 45000, -1000, 2000, -3000, 4000))

    def test_attitude_and_time_sync_decode(self):
        payload = struct.pack("<IHhhhhiiii", 123, 0x85, 32767, 0, -16384, 0,
                              1000, -2000, 3000, -4000)
        frame = FrameParser().feed(encode_frame(MSG_ATTITUDE, 1, payload))[0]
        sample = decode_attitude(frame, 999)
        self.assertAlmostEqual(sample.quaternion_wxyz[2], -0.5, places=3)
        self.assertEqual(sample.yaw_rate_mdeg_s, -4000)
        sync = FrameParser().feed(encode_frame(
            MSG_TIME_SYNC_RESP, 2, struct.pack("<QI", 123456, 789)))[0]
        self.assertEqual(decode_time_sync_response(sync), (123456, 789))

    def test_extended_attitude_diagnostics(self):
        payload = struct.pack(
            "<IHhhhhiiiihhiii", 123, 0x185, 32767, 0, 0, 0,
            1000, -2000, 12, -34, 4012, 3988, 100, -200, 300)
        frame = FrameParser().feed(encode_frame(MSG_ATTITUDE, 3, payload))[0]
        sample = decode_attitude(frame, 456)
        self.assertEqual(sample.temperature_cdeg, 4012)
        self.assertEqual(sample.calibration_temperature_cdeg, 3988)
        self.assertEqual(sample.gyro_offset_urad_s, (100, -200, 300))

    def test_random_chunking(self):
        random.seed(7)
        stream = b"".join(encode_frame(MSG_CONTROL, i, bytes([i & 0xFF])) for i in range(200))
        parser = FrameParser()
        frames = []
        offset = 0
        while offset < len(stream):
            size = random.randint(1, 23)
            frames.extend(parser.feed(stream[offset : offset + size]))
            offset += size
        self.assertEqual([item.sequence for item in frames], list(range(200)))

    def test_duplicate_out_of_order_and_wrap_rule(self):
        self.assertFalse(sequence_is_newer(100, 100))
        self.assertFalse(sequence_is_newer(99, 100))
        self.assertTrue(sequence_is_newer(101, 100))
        self.assertTrue(sequence_is_newer(0, 0xFFFF))
        self.assertTrue(sequence_is_newer(500, 500, initialized=False))

    def test_firmware_timeout_contract(self):
        self.assertEqual(COMMAND_TIMEOUT_MS, 200)
        self.assertGreater(COMMAND_TIMEOUT_MS, 10)


if __name__ == "__main__":
    unittest.main()
