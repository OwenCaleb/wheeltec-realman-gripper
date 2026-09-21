"""使用手册给定的报文和模拟串口验证协议，不接触真实硬件。"""

import unittest
import io
from itertools import repeat
from unittest.mock import patch

from gripper import (Gripper, StateParser, bcc, decode_state, motion_frame, query_frame,
                     print_console_result, console)


class FakeSerial:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.buffer = bytearray()
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, data):
        self.writes.append(data)
        if data[2] == 0:
            self.buffer.extend(next(self.replies, b""))
        return len(data)

    def read(self, count):
        # 模拟 USB/串口分片到达。
        count = min(count, 3)
        chunk = bytes(self.buffer[:count])
        del self.buffer[:count]
        return chunk


class ProtocolTests(unittest.TestCase):
    def test_stationary_feedback_returns_early_without_claiming_completion(self):
        class Clock:
            now = 0.0

            def sleep(self, seconds):
                self.now += seconds

            def monotonic(self):
                return self.now

        clock = Clock()
        stationary = bytes.fromhex("01 01 00 00 00 00 00 c0 c0")
        gripper = Gripper("fake")
        gripper.ser = FakeSerial(repeat(stationary))
        with patch("gripper.time.sleep", clock.sleep), patch("gripper.time.monotonic", clock.monotonic):
            result = gripper.move("open", 180, 3)
        self.assertFalse(result["completion_confirmed"])
        self.assertEqual(result["outcome"], "no_motion_observed")
        self.assertGreaterEqual(result["elapsed_s"], 1.0)
        self.assertLess(result["elapsed_s"], 1.2)
        self.assertEqual(gripper.ser.writes.count(motion_frame("open", 180, 3)), 1)
        output = io.StringIO()
        with patch("sys.stdout", output):
            print_console_result(result)
        self.assertIn("未观察到动作，已返回待命", output.getvalue())

    @patch("gripper.discard_pending_input")
    def test_console_clears_typeahead_even_after_communication_error(self, discard):
        gripper = Gripper("fake")
        output = io.StringIO()
        with patch("builtins.input", side_effect=["open 180 3", "quit"]), \
                patch.object(gripper, "move", side_effect=TimeoutError("no feedback")), \
                patch("sys.stdout", output):
            history = console(gripper, 1, 32)
        discard.assert_called_once()
        self.assertEqual(history, [{"error": "no feedback"}])

    def test_manual_command_vectors(self):
        self.assertEqual(query_frame().hex(" "), "7b 01 00 00 00 00 00 00 00 7a 7d")
        self.assertEqual(motion_frame("open", 1872, 20, open_direction=1).hex(" "),
                         "7b 01 02 01 20 49 20 00 c8 f8 7d")
        self.assertEqual(motion_frame("close", 1872, 20, open_direction=1).hex(" "),
                         "7b 01 02 00 20 49 20 00 c8 f9 7d")

    def test_physically_verified_direction_mapping(self):
        self.assertEqual(motion_frame("close", 180, 3).hex(" "),
                         "7b 01 02 01 20 07 08 00 1e 48 7d")
        self.assertEqual(motion_frame("open", 180, 3).hex(" "),
                         "7b 01 02 00 20 07 08 00 1e 49 7d")
        gripper = Gripper("fake", open_direction=1)
        self.assertEqual(gripper.open_direction, 1)
        with self.assertRaises(ValueError):
            motion_frame("open", 10, open_direction=2)

    def test_manual_feedback_vectors(self):
        moving = decode_state(bytes.fromhex("01 00 00 64 00 00 0e 10 7b"))
        self.assertEqual(moving.speed_rad_s, 10)
        self.assertEqual(moving.motor_angle_deg, 360)
        self.assertFalse(moving.at_target)
        done = decode_state(bytes.fromhex("01 01 00 00 00 00 8c a0 2c"))
        self.assertTrue(done.at_target)
        self.assertEqual(done.motor_angle_deg, 3600)

    def test_signed_32_bit_angle_and_live_negative_feedback(self):
        body = bytes.fromhex("01 01 00 00 80 00 00 00")
        self.assertEqual(decode_state(body + bytes([bcc(body)])).motor_angle_deg, -214748364.8)
        live = decode_state(bytes.fromhex("01 00 00 9a ff ff fc f7 90"))
        self.assertEqual(live.motor_angle_deg, -77.7)
        self.assertEqual(live.speed_rad_s, 15.4)

    def test_corruption_noise_fragmentation_and_multiple_frames(self):
        valid = bytes.fromhex("01 01 00 00 00 00 8c a0 2c")
        parser = StateParser()
        self.assertEqual(parser.feed(b"\xff\xfe" + valid[:4]), [])
        states = parser.feed(valid[4:] + valid[:-1] + b"\xff" + valid + valid)
        self.assertEqual(len(states), 3)
        self.assertTrue(all(s.at_target for s in states))

    def test_outbound_echo_is_not_status(self):
        parser = StateParser()
        self.assertEqual(parser.feed(query_frame() + motion_frame("open", 10, 1)), [])

    def test_reject_invalid_parameters(self):
        for angle in (0, -1, 6553.6, "NaN", "Infinity", "0.15"):
            with self.subTest(angle=angle), self.assertRaises(ValueError):
                motion_frame("open", angle)
        for speed in (0, 42.1, "NaN", "0.15"):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                motion_frame("open", 10, speed)

    def test_no_motion_if_feedback_missing_or_motor_running(self):
        moving = bytes.fromhex("01 00 00 64 00 00 0e 10 7b")
        idle_not_at_target = bytes.fromhex("01 00 00 00 00 00 00 00 01")
        for replies in ([], [moving, moving], [idle_not_at_target, idle_not_at_target]):
            gripper = Gripper("fake", timeout=0.001)
            gripper.ser = FakeSerial(replies)
            with self.assertRaises((TimeoutError, RuntimeError)):
                gripper.move("open", 10)
            self.assertTrue(all(data == query_frame() for data in gripper.ser.writes))

    @patch("gripper.time.sleep")
    def test_motion_is_sent_once_and_completion_requires_change(self, _):
        before = bytes.fromhex("01 01 00 00 00 00 00 00 00")
        after_body = bytes.fromhex("01 01 00 00 00 00 00 64")
        after = after_body + bytes([bcc(after_body)])
        gripper = Gripper("fake", timeout=0.001)
        gripper.ser = FakeSerial([before, before, before, after])
        result = gripper.move("open", 10)
        self.assertTrue(result["completion_confirmed"])
        self.assertEqual(result["motor_angle_delta_deg"], 10.0)
        self.assertEqual(result["requested_motor_angle_deg"], 10.0)
        self.assertEqual(result["completion_scope"], "motor_feedback_only")
        self.assertEqual(gripper.ser.writes.count(motion_frame("open", 10)), 1)
        gripper.ser = FakeSerial([before, before, before])
        result = gripper.move("open", 10)
        self.assertFalse(result["completion_confirmed"])
        self.assertEqual(result["motor_angle_delta_deg"], 0.0)
        self.assertEqual(gripper.ser.writes.count(motion_frame("open", 10)), 1)


if __name__ == "__main__":
    unittest.main()
