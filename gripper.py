#!/usr/bin/env python3
"""WHEELTEC 睿尔曼柔性夹爪 USB 协议；不适用于 MS42DC 或 RS485。"""

import argparse
from contextlib import nullcontext
from dataclasses import asdict, dataclass
from datetime import datetime
from decimal import Decimal, InvalidOperation
import json
from pathlib import Path
import shlex
import sys
import termios
import time

import serial
from serial.tools import list_ports


CONFIG_PATH = Path(__file__).with_name("config.json")


def bcc(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


def integer(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} 必须是 {low}..{high} 的整数")
    return value


def scaled_tenths(value, maximum, name):
    try:
        number = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"{name} 必须是数值") from exc
    if not number.is_finite() or not Decimal("0.1") <= number <= Decimal(str(maximum)):
        raise ValueError(f"{name} 范围为 0.1..{maximum}")
    scaled = number * 10
    if scaled != scaled.to_integral_value():
        raise ValueError(f"{name} 最多支持一位小数")
    return int(scaled)


def frame(payload):
    return payload + bytes([bcc(payload), 0x7D])


def query_frame(device_id=1):
    integer(device_id, 1, 255, "设备地址")
    return frame(bytes([0x7B, device_id, 0, 0, 0, 0, 0, 0, 0]))


def motion_frame(direction, angle_deg, speed_rad_s=1.0, device_id=1, subdivision=32,
                 open_direction=0):
    integer(device_id, 1, 255, "设备地址")
    if direction not in ("open", "close"):
        raise ValueError("方向必须是 open 或 close")
    integer(open_direction, 0, 1, "张开方向位")
    if type(subdivision) is not int or subdivision not in (2, 4, 8, 16, 32):
        raise ValueError("细分值必须是 2、4、8、16、32")
    angle = scaled_tenths(angle_deg, 6553.5, "电机角度（度）")
    speed = scaled_tenths(speed_rad_s, 42, "电机速度（rad/s）")
    direction_bit = open_direction if direction == "open" else 1 - open_direction
    payload = bytes([0x7B, device_id, 0x02, direction_bit, subdivision])
    payload += angle.to_bytes(2, "big") + speed.to_bytes(2, "big")
    return frame(payload)


@dataclass(frozen=True)
class State:
    device_id: int
    at_target: bool
    speed_rad_s: float
    motor_angle_deg: float
    raw_hex: str


def decode_state(data, device_id=1):
    if len(data) != 9:
        raise ValueError("反馈必须为 9 字节")
    if data[0] != device_id or data[1] not in (0, 1):
        raise ValueError("反馈设备地址或到位标志不符")
    if bcc(data[:8]) != data[8]:
        raise ValueError("反馈 BCC 校验失败")
    return State(data[0], bool(data[1]), int.from_bytes(data[2:4], "big") / 10,
                 int.from_bytes(data[4:8], "big", signed=True) / 10, data.hex(" "))


class StateParser:
    """固定长度反馈没有帧头；按地址、标志和 BCC 重新同步，保留不完整帧。"""

    def __init__(self, device_id=1):
        self.device_id = device_id
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        states = []
        while len(self.buffer) >= 9:
            try:
                state = decode_state(bytes(self.buffer[:9]), self.device_id)
            except ValueError:
                del self.buffer[0]
            else:
                del self.buffer[:9]
                states.append(state)
        return states


def available_ports():
    return [{"device": p.device, "description": p.description,
             "serial_number": p.serial_number, "vid": p.vid, "pid": p.pid,
             "hwid": p.hwid} for p in list_ports.comports()]


def resolve_port(port, serial_number):
    if port != "auto":
        return port
    matches = [p for p in available_ports() if p["serial_number"] == serial_number]
    if len(matches) != 1:
        raise RuntimeError(f"未找到唯一的夹爪串口（USB 序列号 {serial_number}）。"
                           "运行 ports 查看设备；必要时用 --port 指定。")
    return matches[0]["device"]


class Gripper:
    def __init__(self, port, baudrate=115200, device_id=1, timeout=2.0, startup_delay=2.0,
                 open_direction=0, full_travel_deg=1872.0, switch_speed_rad_s=10.0,
                 subdivision=32, verbose=False):
        self.port = port
        self.baudrate = baudrate
        self.device_id = integer(device_id, 1, 255, "设备地址")
        self.timeout = timeout
        self.startup_delay = startup_delay
        self.open_direction = integer(open_direction, 0, 1, "张开方向位")
        # 在打开串口前检查完整开合配置。
        motion_frame("open", full_travel_deg, switch_speed_rad_s, device_id, subdivision,
                     self.open_direction)
        self.full_travel_deg = full_travel_deg
        self.switch_speed_rad_s = switch_speed_rad_s
        self.subdivision = subdivision
        self.verbose = verbose
        self.ser = None

    def __enter__(self):
        self.ser = serial.Serial(port=None, baudrate=self.baudrate, bytesize=8,
                                 parity="N", stopbits=1, timeout=0.05,
                                 write_timeout=self.timeout, exclusive=True,
                                 xonxoff=False, rtscts=False, dsrdtr=False)
        # 请求停用 DTR/RTS。驱动打开端口时仍可能产生瞬态，不能保证控制板不复位。
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.port = self.port
        self.ser.open()
        time.sleep(self.startup_delay)
        return self

    def __exit__(self, *_):
        self.ser.close()

    def send(self, data):
        if self.verbose and data[2] != 0:
            print(f"TX {data.hex(' ')}", file=sys.stderr, flush=True)
        if self.ser.write(data) != len(data):
            raise IOError("串口指令未完整写入；不会自动重发运动指令")

    def status(self, timeout_s=None):
        self.ser.reset_input_buffer()
        parser = StateParser(self.device_id)
        received = bytearray()
        self.send(query_frame(self.device_id))
        deadline = time.monotonic() + (self.timeout if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            chunk = self.ser.read(min(max(self.ser.in_waiting, 1), 1024))
            received.extend(chunk)
            states = parser.feed(chunk)
            if states:
                return states[-1]
        raw = bytes(received[-128:]).hex(" ") or "无数据"
        raise TimeoutError(f"未收到有效的 9 字节状态反馈，RX={raw}。"
                           "检查独立供电、115200 波特率和固件版本；"
                           "厂家记录注明 2026-03-20 固件恢复了数据反馈。")

    def move(self, direction, angle_deg, speed_rad_s=1.0, subdivision=32, wait_s=10.0):
        command = motion_frame(direction, angle_deg, speed_rad_s, self.device_id, subdivision,
                               self.open_direction)
        before = self.status()
        time.sleep(0.1)
        settled = self.status()
        if (before.speed_rad_s != 0 or settled.speed_rad_s != 0
                or not before.at_target or not settled.at_target
                or abs(before.motor_angle_deg - settled.motor_angle_deg) > 0.5):
            raise RuntimeError("连续反馈未确认夹爪静止到位，本次未发送新运动指令；"
                               f"反馈：{asdict(before)} -> {asdict(settled)}")
        before = settled
        result = {"command": direction, "tx_hex": command.hex(" "),
                  "direction_bit": command[3],
                  "requested_motor_angle_deg": float(angle_deg),
                  "requested_speed_rad_s": float(speed_rad_s),
                  "before": asdict(before), "after": None,
                  "motor_angle_delta_deg": None,
                  "completion_scope": "motor_feedback_only",
                  "motion_observed": False, "completion_confirmed": False}
        self.send(command)  # 每次调用只发送一次运动指令。
        started = time.monotonic()
        deadline = started + wait_s
        try:
            while time.monotonic() < deadline:
                time.sleep(0.1)
                state = self.status()
                result["after"] = asdict(state)
                result["motor_angle_delta_deg"] = round(state.motor_angle_deg - before.motor_angle_deg, 1)
                if (state.motor_angle_deg != before.motor_angle_deg
                        or state.speed_rad_s != 0 or not state.at_target):
                    result["motion_observed"] = True
                if result["motion_observed"] and state.at_target and state.speed_rad_s == 0:
                    result["completion_confirmed"] = True
                    break
                if (not result["motion_observed"] and state.at_target and state.speed_rad_s == 0
                        and time.monotonic() - started >= 1.0):
                    # 一秒内持续收到静止到位反馈且角度未变，明确返回无动作，不等满十秒。
                    break
        except (TimeoutError, serial.SerialException, OSError) as exc:
            result["feedback_error"] = str(exc)
        result["elapsed_s"] = round(time.monotonic() - started, 2)
        result["outcome"] = (
            "feedback_error" if "feedback_error" in result else
            "target_reported" if result["completion_confirmed"] else
            "no_motion_observed" if not result["motion_observed"] else "monitor_timeout"
        )
        if not result["completion_confirmed"]:
            result["note"] = ("运动指令已发送，但未确认动作完成；不会重发。"
                              "程序退出或超时不等于夹爪停止，请观察实物。")
            if result["after"] is not None and not result["motion_observed"]:
                result["note"] = ("没有观察到电机角度或运动状态变化；可能已到该方向端点，"
                                  "也可能指令未被执行，不能据此确定原因。未自动重发。")
        return result

    def command(self, command):
        """在现有连接中接收 open/close 指令，执行配置中的完整开合行程。"""
        if command not in ("open", "close"):
            raise ValueError("开关指令必须为 'open' 或 'close'")
        # 最低速时长可能较长；根据配置行程估算监测时间，最高约 120 秒。
        wait_s = min(120.0, max(10.0, float(self.full_travel_deg) * 3.141592653589793
                               / 180 / float(self.switch_speed_rad_s) + 5))
        return self.move(command, self.full_travel_deg, self.switch_speed_rad_s,
                         self.subdivision, wait_s)

    def set_open(self, is_open):
        """True 张开，False 闭合；保持同一个串口连接调用即可。"""
        if type(is_open) is not bool:
            raise ValueError("set_open 接受布尔值 True/False")
        return self.command("open" if is_open else "close")

    def open(self):
        return self.command("open")

    def close(self):
        return self.command("close")


def connect(config_path=CONFIG_PATH):
    """供其他 Python 程序使用：with connect() as gripper: gripper.set_open(True)。"""
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    return Gripper(resolve_port(config["port"], config["usb_serial_number"]),
                   config["baudrate"], config["device_id"], config["response_timeout_s"],
                   config.get("startup_delay_s", 2), config.get("open_direction", 0),
                   config.get("full_travel_deg", 1872), config.get("switch_speed_rad_s", 10),
                   config["subdivision"])


def positive_time(value):
    try:
        number = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError("时间参数必须是数值") from exc
    if not 0.1 <= number <= 120:
        raise ValueError("时间参数必须在 0.1..120 秒之间")
    return number


def discard_pending_input():
    """只清空真实终端中的预输入，避免等待动作期间键入的命令排队执行。"""
    if sys.stdin.isatty():
        try:
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
        except (OSError, termios.error):
            pass


def print_console_result(result, json_output=False):
    if json_output:
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        return
    if result["command"] == "status":
        state = result["state"]
        print(f"电机角度 {state['motor_angle_deg']:.1f}°，速度 {state['speed_rad_s']:.1f} rad/s，"
              f"控制器{'已到位' if state['at_target'] else '未到位'}。", flush=True)
        return
    if result["completion_confirmed"]:
        print(f"电机反馈已到位：{result['before']['motor_angle_deg']:.1f}° → "
              f"{result['after']['motor_angle_deg']:.1f}°，"
              f"实际变化 {result['motor_angle_delta_deg']:+.1f}°。", flush=True)
    elif result["outcome"] == "no_motion_observed":
        print("未观察到动作，已返回待命；可能已在该方向端点，不能仅凭反馈确定。", flush=True)
    else:
        print("监测结束，但动作完成未确认。" + result.get("feedback_error", result.get("note", "")),
              flush=True)


def console(gripper, speed, subdivision, json_output=False):
    print("已保持串口连接。open 张开、close 闭合、status 状态、quit 退出。")
    print("可选：open 角度 [速度] / close 角度 [速度] 进行指定行程点动。")
    print("角度单位为电机度，速度单位为 rad/s。退出不发送停止指令。")
    print("10 度仅为电机约 1/36 圈；控制器报告到位不代表爪口已全开/全关。")
    print(f"当前方向映射：open={gripper.open_direction}（张开），"
          f"close={1 - gripper.open_direction}（闭合）。")
    print("每次只输入一条命令，等待 gripper> 再输入下一条；动作等待期间的终端输入会被清除。")
    history = []
    while True:
        try:
            words = shlex.split(input("gripper> "))
            if not words:
                continue
            if words == ["quit"]:
                break
            if words == ["status"]:
                print("正在读取状态…", flush=True)
                try:
                    result = {"command": "status", "state": asdict(gripper.status())}
                finally:
                    discard_pending_input()
            elif words[0] in ("open", "close") and len(words) in (1, 2, 3):
                print("正在检查并等待动作反馈；若持续静止无变化，约 1 秒返回…", flush=True)
                try:
                    if len(words) == 1:
                        result = gripper.command(words[0])
                    else:
                        result = gripper.move(words[0], words[1], words[2] if len(words) == 3 else speed,
                                              subdivision)
                finally:
                    discard_pending_input()
            else:
                print("命令格式：open | close | status | open 180 3 | close 180 3 | quit")
                continue
            history.append(result)
            print_console_result(result, json_output)
        except (ValueError, RuntimeError, OSError, serial.SerialException) as exc:
            result = {"error": str(exc)}
            history.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
        except (EOFError, KeyboardInterrupt):
            print("\n退出串口会话；未发送停止指令。")
            break
    return history


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument("--port", help="串口路径；默认按配置中的 USB 序列号查找")
    parser.add_argument("--timeout", type=float, help="每次反馈的等待时间，秒")
    parser.add_argument("--output", type=Path, help="保存本次结果 JSON")
    parser.add_argument("--dry-run", action="store_true", help="仅显示指令，不打开串口")
    parser.add_argument("--verbose", action="store_true", help="打印串口运动报文")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("ports", help="列出主机串口")
    console_parser = commands.add_parser("console", help="保持同一串口连接，交互控制")
    console_parser.add_argument("--json", action="store_true", help="在控制台显示完整 JSON 反馈")
    keyboard_parser = commands.add_parser("keyboard", help="按住 o 张开、c 闭合，不用回车")
    keyboard_parser.add_argument("--step", type=float,
                                 help="前方最多保留的电机行程，默认 180 度；长按时提前续发")
    keyboard_parser.add_argument("--speed", type=float, help="电机速度 rad/s，默认读取配置")
    status = commands.add_parser("status", help="查询角度、速度和到位标志；不发送运动指令")
    status.add_argument("--samples", type=int, default=1)
    for direction in ("open", "close"):
        move = commands.add_parser(direction, help="张开" if direction == "open" else "闭合")
        move.add_argument("--angle", help="省略时完整开合；指定时按电机角度点动")
        move.add_argument("--speed", help="电机速度 rad/s；默认读取配置")
        move.add_argument("--wait", type=float, help="最多监测动作多少秒")
    args = parser.parse_args(argv)
    result = {"observed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
              "protocol": "wheeltec_realman_usb", "command": args.command}
    exit_code = 0
    keyboard_source = nullcontext()
    try:
        if args.command == "ports":
            result["ports"] = available_ports()
        else:
            config = json.loads(args.config.read_text(encoding="utf-8"))
            device_id = integer(config["device_id"], 1, 255, "设备地址")
            timeout = positive_time(args.timeout if args.timeout is not None
                                    else config["response_timeout_s"])
            baudrate = integer(config["baudrate"], 1, 4000000, "波特率")
            speed = getattr(args, "speed", None) or config["default_speed_rad_s"]
            subdivision = config["subdivision"]
            open_direction = integer(config.get("open_direction", 0), 0, 1, "张开方向位")
            startup_delay = positive_time(config.get("startup_delay_s", 2.0))
            full_travel = config.get("full_travel_deg", 1872.0)
            switch_speed = config.get("switch_speed_rad_s", 10.0)
            if args.command == "status":
                integer(args.samples, 1, 1000, "采样数")
                command = query_frame(device_id)
            elif args.command in ("open", "close"):
                angle = args.angle if args.angle is not None else full_travel
                speed = args.speed if args.speed is not None else (
                    switch_speed if args.angle is None else config["default_speed_rad_s"])
                command = motion_frame(args.command, angle, speed, device_id, subdivision,
                                       open_direction)
                wait_s = positive_time(args.wait) if args.wait is not None else min(
                    120.0, max(10.0, float(angle) * 3.141592653589793 / 180 / float(speed) + 5))
            elif args.command == "keyboard":
                from keyboard_control import PynputKeys, validate_jog_settings
                step = args.step if args.step is not None else config.get("keyboard_step_deg", 180)
                speed = args.speed if args.speed is not None else config.get("keyboard_speed_rad_s", 10)
                validate_jog_settings(step, speed)
                keyboard_source = PynputKeys()
            if args.dry_run:
                if args.command in ("console", "keyboard"):
                    raise ValueError("交互模式不支持 --dry-run；请预览具体 open/close 指令")
                result.update(dry_run=True, tx_hex=command.hex(" "))
            else:
                port = resolve_port(args.port or config["port"], config["usb_serial_number"])
                result["port"] = port
                # 先启动键盘监听，再连接控制板；桌面不可用时不打开串口。
                with keyboard_source as keys, Gripper(
                        port, baudrate, device_id, timeout, startup_delay, open_direction,
                        full_travel, switch_speed, subdivision, args.verbose) as gripper:
                    if args.command == "status":
                        result["states"] = []
                        for _ in range(args.samples):
                            result["states"].append(asdict(gripper.status()))
                            if args.samples > 1:
                                time.sleep(0.1)
                    elif args.command == "console":
                        result["history"] = console(gripper, speed, subdivision, args.json)
                        exit_code = int(any("error" in row or "note" in row for row in result["history"]))
                    elif args.command == "keyboard":
                        from keyboard_control import run_keyboard
                        result.update(run_keyboard(gripper, keys, step, speed, subdivision))
                        exit_code = 1 if result.get("error") else (130 if result["interrupted"] else 0)
                    else:
                        result.update(gripper.move(args.command, angle, speed, subdivision, wait_s))
                        exit_code = 0 if result["completion_confirmed"] else 2
    except (ValueError, KeyError, OSError, RuntimeError, serial.SerialException) as exc:
        result["error"] = str(exc)
        exit_code = 1
    except KeyboardInterrupt:
        result["error"] = "用户中断程序；这不会发送停止指令，也不保证夹爪停止。"
        exit_code = 130
    document = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(document, encoding="utf-8")
    if args.command not in ("console", "keyboard") or getattr(args, "json", False) or "error" in result:
        print(document, end="", flush=True)
    else:
        print("串口会话已结束。" + (f"记录已保存至 {args.output}。" if args.output else ""), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
