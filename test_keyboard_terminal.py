"""不连接真实硬件：验证终端回显恢复及实际控制循环的长按/释放/退出。"""
import io
import os
import pty
import termios
import unittest
from unittest.mock import patch

from gripper import State
from keyboard_control import HeldKeys, QuietTerminal, run_keyboard


class KeyboardIntegrationTests(unittest.TestCase):
    def test_terminal_restored_on_exception(self):
        master, slave = pty.openpty()
        try:
            original = termios.tcgetattr(slave)
            with os.fdopen(os.dup(slave), "r") as terminal, patch("sys.stdin", terminal):
                with self.assertRaises(RuntimeError):
                    with QuietTerminal():
                        self.assertFalse(termios.tcgetattr(slave)[3] & termios.ECHO)
                        raise RuntimeError("exit")
                self.assertEqual(termios.tcgetattr(slave), original)
        finally:
            os.close(master)
            os.close(slave)

    def test_loop_hold_release_reverse_and_quit_without_repeated_presses(self):
        class Clock:
            now = 0
            def sleep(self, seconds):
                self.now += seconds
            def monotonic(self):
                return self.now
        clock = Clock()

        class ScriptedKeys(HeldKeys):
            events = [(0.1, "press", "c"), (.8, "release", "c"),
                      (1.2, "press", "o"), (1.8, "press", "q")]
            def snapshot(self, now=None):
                while self.events and clock.now >= self.events[0][0]:
                    _, kind, key = self.events.pop(0)
                    getattr(self, kind)(key)
                return super().snapshot(now)

        class FakeGripper:
            device_id = 1
            open_direction = 0
            angle = 500
            target = 500
            last = 0
            writes = []
            moving_when_sent = []
            def status(self, timeout_s=None):
                remaining = self.target - self.angle
                travel = min(abs(remaining), 570 * (clock.now - self.last))
                self.angle += travel if remaining > 0 else -travel
                self.last = clock.now
                moving = abs(self.target - self.angle) > .01
                return State(1, not moving, 10 if moving else 0, self.angle, "")
            def send(self, data):
                self.moving_when_sent.append(not self.status().at_target)
                self.writes.append(data)
                self.target += (1 if data[3] else -1) * int.from_bytes(data[5:7], "big") / 10

        gripper = FakeGripper()
        with patch("keyboard_control.time.monotonic", clock.monotonic), \
                patch("keyboard_control.time.sleep", clock.sleep), \
                patch("sys.stdout", io.StringIO()):
            result = run_keyboard(gripper, ScriptedKeys())
        self.assertNotIn("error", result)
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual({p[3] for p in gripper.writes}, {1, 0})
        self.assertTrue(any(gripper.moving_when_sent))
        self.assertEqual(len(result["history"]), 2)
        self.assertTrue(all(row["outcome"] == "target_reported" for row in result["history"]))
        self.assertGreater(result["history"][0]["motor_angle_delta_deg"], 300)


if __name__ == "__main__":
    unittest.main()
