"""需要隔离的 X11 测试桌面，绝不向用户桌面注入测试按键。"""
import os
import subprocess
import sys
import unittest
from pathlib import Path


@unittest.skipUnless(os.environ.get("GRIPPER_TEST_DISPLAY"), "需显式指定隔离的 GRIPPER_TEST_DISPLAY")
class PynputIntegrationTest(unittest.TestCase):
    def test_real_press_hold_release_callbacks(self):
        program = r'''
import time
from keyboard_control import PynputKeys
from pynput import keyboard

def wait_for(keys, expected):
    end = time.monotonic() + 2
    while time.monotonic() < end:
        if keys.snapshot()[0] == expected:
            return
        time.sleep(.01)
    raise AssertionError((expected, keys.snapshot()))

with PynputKeys() as keys:
    keys.activate()
    sender = keyboard.Controller()
    sender.press('c')
    wait_for(keys, 'close')
    time.sleep(.15)  # 不需要重复发送 press。
    assert keys.snapshot()[0] == 'close'
    sender.release('c')
    wait_for(keys, None)
    sender.press('o')
    wait_for(keys, 'open')
    sender.press(keyboard.Key.space)
    wait_for(keys, None)
    sender.release(keyboard.Key.space)
    sender.release('o')
    sender.press('q')
    end = time.monotonic() + 2
    while not keys.snapshot()[2] and time.monotonic() < end:
        time.sleep(.01)
    sender.release('q')
    assert keys.snapshot()[2]
'''
        env = dict(os.environ, DISPLAY=os.environ["GRIPPER_TEST_DISPLAY"])
        subprocess.run([sys.executable, "-c", program], cwd=Path(__file__).parent,
                       env=env, check=True, timeout=10, capture_output=True, text=True)
