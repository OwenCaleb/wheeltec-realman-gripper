"""VLA demo 离线测试；不连接夹爪。"""

import io
import unittest
from unittest.mock import call, patch

from demo_vla import main


class VlaDemoTests(unittest.TestCase):
    @patch("demo_vla.connect")
    def test_o_p_actions_share_connection_and_skip_repeats(self, connect):
        gripper = connect.return_value.__enter__.return_value
        gripper.set_open.return_value = {"completion_confirmed": True, "outcome": "target_reported"}
        with patch("sys.stdout", io.StringIO()):
            self.assertEqual(main(["o", "o", "p", "p", "o"]), 0)
        connect.assert_called_once_with()
        self.assertEqual(gripper.set_open.call_args_list, [call(True), call(False), call(True)])
        connect.return_value.__exit__.assert_called_once()

    @patch("demo_vla.connect")
    def test_unconfirmed_command_does_not_continue_sequence(self, connect):
        gripper = connect.return_value.__enter__.return_value
        gripper.set_open.return_value = {"completion_confirmed": False, "outcome": "no_motion_observed"}
        with patch("sys.stdout", io.StringIO()):
            self.assertEqual(main(["p", "o"]), 2)
        gripper.set_open.assert_called_once_with(False)
        connect.return_value.__exit__.assert_called_once()

    @patch("demo_vla.connect")
    def test_invalid_action_does_not_connect(self, connect):
        for action in ("0", "1", "open", "close", "c", "0.5"):
            with self.subTest(action=action), patch("sys.stderr", io.StringIO()), \
                    self.assertRaises(SystemExit) as exit:
                main([action])
            self.assertEqual(exit.exception.code, 2)
        connect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
