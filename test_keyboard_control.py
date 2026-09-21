import io
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from gripper import Gripper, State
from keyboard_control import ContinuousController, HeldKeys, run_keyboard, validate_jog_settings


def state(angle=19.2, speed=0, at_target=True):
    return State(1, at_target, speed, angle, "")


class KeyboardAndSwitchTests(unittest.TestCase):
    def test_simple_switch_commands_and_boolean_api(self):
        gripper = Gripper("fake", full_travel_deg=1872, switch_speed_rad_s=10)
        with patch.object(gripper, "move", return_value={}) as move:
            gripper.command("open")
            self.assertEqual(move.call_args.args[:4], ("open", 1872, 10, 32))
            gripper.set_open(False)
            self.assertEqual(move.call_args.args[0], "close")
            gripper.open()
            gripper.close()
            self.assertEqual(move.call_count, 4)
            for invalid in ("true", 1, None):
                with self.assertRaises(ValueError):
                    gripper.set_open(invalid)
            with self.assertRaises(ValueError):
                gripper.command("toggle")
            self.assertEqual(move.call_count, 4)

    def test_hold_without_autorepeat_and_real_release(self):
        keys = HeldKeys()
        keys.activate()
        keys.press(SimpleNamespace(char="C"), now=1)
        self.assertEqual(keys.snapshot(1), ("close", 1, False))
        self.assertEqual(keys.snapshot(100), ("close", 1, False))
        keys.release("c", now=101)
        self.assertIsNone(keys.snapshot(101.03)[0])
        keys.press("o", now=102)
        self.assertEqual(keys.snapshot(102), ("open", 2, False))

    def test_x11_autorepeat_does_not_release_or_queue_motion(self):
        keys = HeldKeys()
        keys.activate()
        keys.press("c", 1)
        for t in (2, 3, 4):
            keys.release("c", t)
            self.assertEqual(keys.snapshot(t + .001)[0], "close")
            keys.press("c", t + .002)
            self.assertEqual(keys.snapshot(t + .01), ("close", 1, False))
        keys.release("c", 5)
        self.assertIsNone(keys.snapshot(5.03)[0])

    def test_pause_both_keys_and_ctrl_require_release(self):
        for pause in (" ", "o", SimpleNamespace(name="ctrl_l")):
            with self.subTest(pause=pause):
                keys = HeldKeys()
                keys.activate()
                keys.press("c", 1)
                keys.press(pause, 2)
                self.assertIsNone(keys.snapshot(2)[0])
                keys.release(pause, 3)
                keys.press("c", 4)  # 自动重复不能解除暂停。
                self.assertIsNone(keys.snapshot(4)[0])
                keys.release("c", 5)
                keys.snapshot(5.1)
                keys.press("c", 6)
                self.assertEqual(keys.snapshot(6)[0], "close")

    def test_key_held_before_ready_does_not_start(self):
        keys = HeldKeys()
        keys.press("c")
        keys.activate()
        self.assertIsNone(keys.snapshot()[0])
        keys.press("c")
        self.assertIsNone(keys.snapshot()[0])
        keys.release("c", 0)
        keys.snapshot(1)
        keys.press("c", 2)
        self.assertEqual(keys.snapshot(2)[0], "close")

    def test_quit_overrides_held_key(self):
        keys = HeldKeys()
        keys.activate()
        keys.press("c", 1)
        keys.press(SimpleNamespace(name="esc"), 2)
        self.assertEqual(keys.snapshot(2), (None, 1, True))
        keys.press("o", 3)
        self.assertIsNone(keys.snapshot(3)[0])

    def test_refill_before_target_with_bounded_remaining_travel(self):
        ctl = ContinuousController()
        packet = ctl.request("close", 1, state(), 0)
        self.assertEqual(int.from_bytes(packet[5:7], "big"), 1800)
        self.assertIsNone(ctl.request("close", 1, state(99.2, 10, False), .15))
        packet = ctl.request("close", 1, state(119.2, 10, False), .2)
        self.assertEqual(int.from_bytes(packet[5:7], "big"), 900)
        self.assertLessEqual(ctl.active["commanded"] - 100, ctl.step)
        ctl.observe(state(119.2, 10, False), .2)
        self.assertIsNone(ctl.request(None, 1, state(119.2, 10, False), .21))
        # 松键后即使仍在运动，也绝不续发。
        self.assertIsNone(ctl.request(None, 1, state(219.2, 10, False), .4))
        ctl.observe(state(289.2), .6)
        self.assertIsNone(ctl.active)
        self.assertEqual(ctl.history[0]["packets"], 2)

    def test_reverse_waits_for_remaining_motion(self):
        ctl = ContinuousController()
        ctl.request("close", 1, state(), 0)
        self.assertIsNone(ctl.request("open", 2, state(50, 10, False), .1))
        ctl.observe(state(50, 10, False), .1)
        self.assertIsNone(ctl.request("open", 2, state(100, 10, False), .2))
        ctl.observe(state(198), .4)
        self.assertEqual(ctl.request("open", 2, state(198), .41)[3], 0)

    def test_endpoint_blocks_same_hold(self):
        ctl = ContinuousController()
        ctl.request("close", 1, state(), 0)
        ctl.observe(state(), .6)
        self.assertIsNone(ctl.active)
        self.assertIsNone(ctl.request("close", 1, state(), 1))
        self.assertIsNotNone(ctl.request("open", 2, state(), 1.1))

    def test_motion_timeout(self):
        ctl = ContinuousController(180, 10)
        ctl.request("close", 1, state(), 0)
        ctl.request(None, 1, state(), .1)
        with self.assertRaises(TimeoutError):
            ctl.observe(state(100, 10, False), 2.2)

    def test_external_motion_is_not_overridden(self):
        ctl = ContinuousController()
        self.assertIsNone(ctl.request("close", 1, state(100, 10, False), 0))

    def test_settings_bound_release_travel(self):
        validate_jog_settings(180, 10)
        validate_jog_settings(90, 5)
        for step, speed in ((0, 10), (1872, 10), (180, 0), (180, 1), (180, float("nan"))):
            with self.subTest(step=step, speed=speed), self.assertRaises(ValueError):
                validate_jog_settings(step, speed)

    def test_control_loop_drains_on_listener_loss_or_interrupt(self):
        for reason in (RuntimeError("listener disconnected"), TimeoutError("feedback lost"), KeyboardInterrupt()):
            class LostKeys(HeldKeys):
                def check_alive(self):
                    raise reason
            gripper = Gripper("fake")
            with patch.object(gripper, "send") as send, \
                    patch("keyboard_control.confirm_stopped", return_value=state()) as stopped, \
                    patch("sys.stdout", io.StringIO()):
                result = run_keyboard(gripper, LostKeys())
            send.assert_not_called()
            self.assertEqual(stopped.call_count, 2)
            self.assertTrue(result["stop_confirmed"])
            self.assertEqual(result["interrupted"], isinstance(reason, KeyboardInterrupt))
            if not isinstance(reason, KeyboardInterrupt):
                self.assertIn(str(reason), result["error"])


if __name__ == "__main__":
    unittest.main()
