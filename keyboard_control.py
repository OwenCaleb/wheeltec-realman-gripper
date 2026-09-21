"""pynput 长按控制：按反馈提前续发有限行程，不依赖键盘重复。"""

from dataclasses import asdict
import math
import os
from pathlib import Path
import sys
import termios
import threading
import time

from gripper import motion_frame


def validate_jog_settings(step, speed):
    motion_frame("open", step, speed)
    if not 1 <= step <= 360:
        raise ValueError("键盘步长必须在 1..360 度之间")
    if math.radians(step) / speed > 0.5:
        raise ValueError("键盘剩余行程预计不能超过 0.5 秒；减小 --step 或增大 --speed")


class HeldKeys:
    """监听线程只维护按键状态；所有串口操作均在控制线程完成。"""

    RELEASE_DELAY = 0.025  # 合并 X11 自动重复产生的 release/press 对。

    def __init__(self):
        self.lock = threading.Lock()
        self.pressed = set()
        self.releases = {}
        self.enabled = False
        self.inhibited = False
        self.quitting = False
        self.epoch = 0

    @staticmethod
    def name(key):
        char = key if isinstance(key, str) else getattr(key, "char", None)
        if char:
            return char.lower()
        name = getattr(key, "name", "")
        return {"space": " ", "esc": "q"}.get(name, name)

    def _flush_releases(self, now):
        for key, deadline in list(self.releases.items()):
            if now >= deadline:
                self.pressed.discard(key)
                del self.releases[key]
        if not self.pressed.intersection({"o", "c", "ctrl", "ctrl_l", "ctrl_r"}):
            self.inhibited = False

    def press(self, key, now=None):
        key = self.name(key)
        now = time.monotonic() if now is None else now
        with self.lock:
            self._flush_releases(now)
            self.releases.pop(key, None)
            if key in ("q", "\x1b"):
                self.quitting = True
            if key in (" ", "ctrl", "ctrl_l", "ctrl_r"):
                self.inhibited = True
            if key in ("o", "c") and key not in self.pressed:
                self.epoch += 1
            self.pressed.add(key)
            if {"o", "c"}.issubset(self.pressed):
                self.inhibited = True

    def release(self, key, now=None):
        key = self.name(key)
        now = time.monotonic() if now is None else now
        with self.lock:
            if key in ("o", "c"):
                self.releases[key] = now + self.RELEASE_DELAY
            else:
                self.pressed.discard(key)

    def activate(self):
        with self.lock:
            self._flush_releases(time.monotonic())
            self.enabled = True
            # 初始化时已按住的键必须先松开，再次按下才启动。
            self.inhibited = bool(self.pressed.intersection({"o", "c", "ctrl", "ctrl_l", "ctrl_r"}))

    def snapshot(self, now=None):
        with self.lock:
            self._flush_releases(time.monotonic() if now is None else now)
            direction = None
            if self.enabled and not self.inhibited and not self.quitting:
                if "o" in self.pressed:
                    direction = "open"
                elif "c" in self.pressed:
                    direction = "close"
            return direction, self.epoch, self.quitting

    def check_alive(self):
        pass  # 离线测试可直接使用 HeldKeys，不连接图形桌面。


class PynputKeys(HeldKeys):
    def __enter__(self):
        # 本机只有一个 X11 桌面时，可从 tty 启动并监听本地实体键盘。
        if not os.environ.get("DISPLAY"):
            sockets = sorted(Path("/tmp/.X11-unix").glob("X[0-9]*"))
            if len(sockets) != 1:
                raise RuntimeError("无法确定本地图形桌面；请在主机桌面终端运行，或设置 DISPLAY。")
            os.environ["DISPLAY"] = ":" + sockets[0].name[1:]
        auth = Path(f"/run/user/{os.getuid()}/gdm/Xauthority")
        if not os.environ.get("XAUTHORITY") and auth.is_file():
            os.environ["XAUTHORITY"] = str(auth)
        try:
            from pynput import keyboard
            # 1.8.2 的第二个参数是 injected，不能当作事件时间传入 HeldKeys。
            self.listener = keyboard.Listener(
                on_press=lambda key, injected: self.press(key),
                on_release=lambda key, injected: self.release(key))
            self.listener.start()
            ready = threading.Event()

            def wait_ready():
                self.listener.wait()
                ready.set()

            threading.Thread(target=wait_ready, daemon=True).start()
            if not ready.wait(3) or not self.listener.is_alive():
                # X11 初始化失败时 stop() 本身可能等待 ready，不能阻塞主线程。
                threading.Thread(target=self.listener.stop, daemon=True).start()
                raise RuntimeError("键盘监听器未能就绪")
        except (ImportError, OSError, RuntimeError) as exc:
            raise RuntimeError(f"pynput 无法连接本地图形桌面 {os.environ.get('DISPLAY')}：{exc}。"
                               "请在主机桌面终端运行；依赖为 pynput==1.8.2。") from exc
        print(f"已连接本地键盘（DISPLAY={os.environ['DISPLAY']}，pynput）。", flush=True)
        return self

    def check_alive(self):
        if not self.listener.is_alive():
            raise RuntimeError("键盘监听器已断开")

    def __exit__(self, *_):
        self.listener.stop()
        self.listener.join(timeout=1)


class ContinuousController:
    """在旧目标前半段走完时延长目标，保留至多一步前瞻行程。"""
    def __init__(self, step=180, speed=10, device_id=1, subdivision=32, open_direction=0):
        validate_jog_settings(step, speed)
        self.step = step
        self.refill = round(step / 2, 1)
        self.speed = speed
        self.device_id = device_id
        self.subdivision = subdivision
        self.open_direction = open_direction
        self.active = None
        self.blocked_epoch = None
        self.history = []
        self.message = "待命"

    def packet(self, direction, angle):
        return motion_frame(direction, angle, self.speed, self.device_id,
                            self.subdivision, self.open_direction)

    def request(self, direction, epoch, state, now):
        if self.active:
            a = self.active
            if direction != a["direction"] or epoch != a["epoch"]:
                if not a.get("draining"):
                    a.update(draining=True, release_time=now)
                self.message = "停止续发，等待剩余行程结束"
            if a.get("draining"):
                return None
            progress = a["sign"] * (state.motor_angle_deg - a["before"])
            remaining = a["commanded"] - progress
            if remaining <= self.step - self.refill and now - a["last_send"] >= 0.05:
                a["commanded"] += self.refill
                a["last_send"] = now
                a["packets"] += 1
                return self.packet(direction, self.refill)
            return None
        if direction is None or epoch == self.blocked_epoch:
            return None
        if not state.at_target or state.speed_rad_s != 0:
            self.message = "等待电机静止"
            return None
        data = self.packet(direction, self.step)
        self.active = {"direction": direction, "epoch": epoch, "started": now,
                       "last_send": now, "before": state.motor_angle_deg,
                       "observed": False, "commanded": self.step, "packets": 1,
                       "sign": 1 if data[3] else -1}
        self.message = "持续张开" if direction == "open" else "持续闭合"
        return data

    def observe(self, state, now):
        if not self.active:
            return
        a = self.active
        delta = round(state.motor_angle_deg - a["before"], 1)
        elapsed = now - a["started"]
        a["observed"] |= delta != 0 or state.speed_rad_s != 0 or not state.at_target
        if state.at_target and state.speed_rad_s == 0 and (a["observed"] or elapsed >= 0.5):
            outcome = "target_reported" if a["observed"] else "no_motion_observed"
            self.history.append({"direction": a["direction"], "packets": a["packets"],
                                 "motor_angle_delta_deg": delta, "outcome": outcome,
                                 "elapsed_s": round(elapsed, 3),
                                 "release_to_idle_s": round(now - a["release_time"], 3)
                                 if a.get("draining") else None})
            if not a["observed"]:
                self.blocked_epoch = a["epoch"]
            self.active = None
            self.message = "待命" if a["observed"] else "未观察到动作；松键后可重试或反向"
        elif (a.get("draining") and now - a["release_time"] >= 2) or (
                elapsed > math.radians(a["commanded"]) / self.speed + 3):
            raise TimeoutError("剩余行程未按预期结束，已停止续发；请检查实物")


class QuietTerminal:
    """只关闭字符回显，不使用终端字符进行控制。"""
    def __enter__(self):
        self.original = None
        if sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.original = termios.tcgetattr(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[3] &= ~termios.ECHO
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        return self

    def __exit__(self, *_):
        if self.original is not None:
            termios.tcflush(self.fd, termios.TCIFLUSH)
            termios.tcsetattr(self.fd, termios.TCSANOW, self.original)


def confirm_stopped(gripper, timeout=1.0):
    deadline = time.monotonic() + timeout
    previous = None
    while time.monotonic() < deadline:
        state = gripper.status(timeout_s=0.15)
        if (previous and state.at_target and previous.at_target
                and state.speed_rad_s == previous.speed_rad_s == 0
                and abs(state.motor_angle_deg - previous.motor_angle_deg) <= 0.5):
            return state
        previous = state
        time.sleep(0.06)
    raise TimeoutError("未确认电机静止；请检查实物")


def run_keyboard(gripper, keys, step=180, speed=10, subdivision=32):
    controller = ContinuousController(step, speed, gripper.device_id, subdivision,
                                      gripper.open_direction)
    print(f"按住 o 张开、c 闭合；空格暂停，q / Esc 退出。速度 {speed:g} rad/s。")
    print(f"松键停止续发，最多约 {step:g}° 剩余电机行程会走完（匀速约 {math.radians(step)/speed:.2f} 秒，另有加减速时间）。")
    print("监听本地桌面全局按键，无需回车或系统按键重复。")
    state = None
    interrupted = False
    error = None
    stop_confirmed = False
    try:
        state = confirm_stopped(gripper, timeout=3)
        keys.activate()
        last_poll = last_display = 0.0
        with QuietTerminal():
            while True:
                keys.check_alive()
                direction, epoch, quitting = keys.snapshot()
                now = time.monotonic()
                data = controller.request(None if quitting else direction, epoch, state, now)
                if data is not None:
                    gripper.send(data)
                if now - last_poll >= 0.05:
                    state = gripper.status(timeout_s=0.15)
                    last_poll = time.monotonic()
                    controller.observe(state, last_poll)
                if now - last_display >= 0.1:
                    print(f"\r\033[K电机 {state.motor_angle_deg:8.1f}° | "
                          f"{state.speed_rad_s:4.1f} rad/s | {controller.message}", end="", flush=True)
                    last_display = now
                if quitting and controller.active is None:
                    break
                time.sleep(0.01)
    except KeyboardInterrupt:
        interrupted = True
    except (OSError, RuntimeError, ValueError) as exc:
        error = str(exc)
    finally:
        # 停止续发后监测有限的剩余行程；不发送未经实物验证的停止报文。
        try:
            state = confirm_stopped(gripper, timeout=2)
            stop_confirmed = True
        except (OSError, RuntimeError, ValueError) as exc:
            error = (error + "；" if error else "") + f"退出停止未确认：{exc}"
        print("\n键盘控制已退出；" + ("电机停止已确认。" if stop_confirmed else "停止未确认，请检查实物。"),
              flush=True)
    result = {"history": controller.history, "final_state": asdict(state) if state else None,
              "interrupted": interrupted, "stop_confirmed": stop_confirmed}
    if error:
        result["error"] = error
    return result
