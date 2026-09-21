# WHEELTEC 睿尔曼柔性夹爪 Python 控制

通过 **USB 串口**控制 WHEELTEC 睿尔曼柔性夹爪，提供完整张开/闭合、指定电机行程点动、
`o` / `p` 长按连续调节，以及供其他程序调用的 Python 接口。不依赖 ROS 或机械臂控制器。

这是根据夹爪随附资料实现并在实物上验证的独立工具，并非厂家官方 SDK。

## 适用设备与运行环境

| 项目 | 说明 |
|---|---|
| 夹爪 | WHEELTEC 资料中命名为 **睿尔曼柔性机械爪**的版本及其配套控制板 |
| 主机连接 | 控制板 Type-C USB 接口，USB 转串口控制 |
| 串口参数 | 115200 baud、8 数据位、无校验、1 停止位、无流控 |
| 控制协议 | `7B ... 7D` USB 定位控制帧，控制模式 `02` |
| 电机供电 | 随附手册标注为 10–28 V；USB 不替代独立电机供电 |
| 操作系统 | Linux；已在 Ubuntu 主机验证。代码使用 `termios`，未提供 Windows 支持 |
| Python | Python 3.10+；已在 Python 3.12 验证 |
| 依赖 | `pyserial==3.5`、`pynput==1.8.2` |
| 键盘环境 | 主机本地 X11 图形桌面；console / Python 开关接口不需要桌面 |

**不适用于**同一资料包里的 MS42DC 步进电机版、舵机版柔性夹爪，也不是通用的睿尔曼夹爪驱动。
RS485 的 `EB 90 ...` 协议与这里的 USB 协议不同，不能直接将本程序用于 USB-RS485 适配器。

## 接线与安装

1. 按夹爪配套手册接好独立电源，确认电压和极性。
2. 用 USB 数据线连接控制板 Type-C 接口与 Linux 主机。被测控制板识别为 CH9102 系列 USB 串口，
   VID/PID 为 `1a86:55d4`，串口通常类似 `/dev/ttyACM0`，具体以枚举结果为准。
3. 安装程序并创建本地配置：

```bash
git clone https://github.com/OwenCaleb/wheeltec-realman-gripper.git
cd wheeltec-realman-gripper
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
cp config.example.json config.json
```

启动脚本按以下顺序选择 Python：`GRIPPER_PYTHON` 环境变量、本目录 `.venv/bin/python`、`python3`。
例如已有 Conda 环境时，可以直接使用该环境的 Python：

```bash
GRIPPER_PYTHON=/path/to/conda/env/bin/python bash gripper.sh ports
```

### 选择你的串口

先枚举设备，此命令不打开夹爪连接：

```bash
bash gripper.sh ports
```

编辑 `config.json`，用以下任一种方式指定夹爪：

- 保持 `"port": "auto"`，把 `usb_serial_number` 填为枚举结果中的真实序列号。
- 将 `port` 改为实际设备路径，例如 `/dev/ttyACM0` 或 `/dev/serial/by-id/` 下的稳定路径。
  显式指定路径时不会使用 `usb_serial_number`。

示例配置中的序列号故意留空，需要自行填写。`config.json` 已加入 Git 忽略规则。

先检查报文，再查询反馈：

```bash
# 仅预览报文，不打开串口，不动作
bash gripper.sh --dry-run open

# 查询反馈，不发送业务运动指令
bash gripper.sh status
```

**连接行为：** 被测控制板曾在打开串口时自动张开/回零。程序连接后默认等待 2 秒；
关闭 DTR/RTS 也不能保证驱动打开串口时没有瞬态。因此 `status` 虽不发送运动指令，
建立连接仍可能触发控制板自身动作。连续使用时推荐保持同一 console 或 Python 连接。

## 方式一：直接完整张开 / 闭合

```bash
bash gripper.sh console
```

在 `gripper>` 提示符下，每次输入一条并回车，等待再次出现提示符后再输入下一条：

| 命令 | 动作 |
|---|---|
| `open` | 完整张开 |
| `close` | 完整闭合 |
| `status` | 读取电机角度、速度和到位标志 |
| `open 180 3` | 按张开方向点动 180° 电机行程，速度 3 rad/s |
| `close 180 3` | 按闭合方向点动 180° 电机行程，速度 3 rad/s |
| `quit` | 退出 console 并关闭串口 |

无参数 `open` / `close` 使用配置中的 `full_travel_deg=1872`、`switch_speed_rad_s=10`。
这是在被测夹爪上验证能够完成完整开合的行程配置，不是所有机械结构的通用标定值。
小角度是**电机轴角度**：10° 仅约为电机的 1/36 圈，爪口可能没有明显可见变化。

也支持单次 CLI 命令；每次都会重新连接串口：

```bash
bash gripper.sh open
bash gripper.sh close
bash gripper.sh open --angle 180 --speed 3
```

首次使用另一台夹爪时，先用小行程确认方向。当前默认映射为 **方向位 0 = 张开，1 = 闭合**，
来自实物观察，与随附手册的文字方向相反。如你的设备相反，将 `open_direction` 改为 `1`。

## 方式二：`o` / `p` 键盘连续调节

先退出 console 或其他占用夹爪串口的程序，在**主机本地桌面终端**运行：

```bash
bash gripper.sh keyboard
```

| 按键 | 动作 |
|---|---|
| 按住 `o` | 连续张开 |
| 按住 `p` | 连续闭合 |
| 松开方向键 | 停止续发，剩余有限行程走完 |
| 空格 / 同时按 `o` 和 `p` | 暂停；松开方向键后再按才能继续 |
| `q` / Esc | 停止续发，等待剩余行程结束后退出 |
| Ctrl+C | 停止续发，尝试确认剩余行程结束后退出 |

使用 **pynput 1.8.2** 监听真实的按下和松开事件，不需要回车或系统按键自动重复。
闭合键为 `p`，字母 `c` 已取消夹爪绑定，便于与机械臂键盘控制同时使用。
控制线程在上一段行程结束前根据角度反馈延长目标，因此长按时不会逐段等待到位。
反向时先等待剩余行程结束；启动时已经按住的方向键需要松开后再按。

默认 `--step 180 --speed 10`。`step` 表示前方保留的电机行程，长按累计行程可以超过它。
需要更细的控制或更短的松键尾程时：

```bash
bash gripper.sh keyboard --step 90 --speed 10
```

参数限制为步长 1–360°，且 `步长 × π / 180 / 速度 ≤ 0.5 秒`。

**松键不是即时硬件停止。** 默认剩余行程按匀速计算约 0.31 秒，另有控制器加减速和反馈延迟。
两次实物测试中，控制循环识别松键后约 0.215 秒、0.409 秒收到静止到位反馈，不能将此时间视为保证值。
上位机停止报文和 USB 零速度模式在被测设备上未能可靠中止定位行程，本程序采用有限行程续发，
没有使用这些候选停止报文。串口断开或程序被强制终止也不保证电机立即停止。

键盘监听范围是本地桌面全局，切换窗口后仍有效。普通 SSH 终端输入的字符不等于主机实体键盘事件。
没有 `DISPLAY` 时，程序会尝试连接本机唯一的 X11 桌面；无法连接则在打开串口前退出。
Wayland 桌面的全局监听未验证，推荐使用 X11 会话。

## VLA 最简开关 demo

[demo_vla.py](demo_vla.py) 的核心只有一句 `gripper.set_open(is_open)`。
一条指令即可完整张开或闭合，不需要指定角度、速度或监听键盘：

```bash
.venv/bin/python demo_vla.py o    # 完整张开
.venv/bin/python demo_vla.py p    # 完整闭合
```

demo **只接受 `o` / `p`**。需要顺序测试多条指令时，可保持一个连接执行：

```bash
# 依次张开、闭合、张开；重复的 p 会被跳过
.venv/bin/python demo_vla.py o p p o
```

每次单独启动脚本会重新连接串口，适合快速验证。真实 VLA 循环应在循环外连接一次，
状态发生变化时再下发动作；可以直接采用 demo 中的循环，或参考：

```python
from gripper import connect

vla_actions = ["o", "o", "p", "p", "o"]  # 将模型的夹爪动作映射为 o/p 指令流
with connect() as gripper:
    last_action = None
    for action in vla_actions:
        if action not in ("o", "p"):
            raise ValueError("夹爪指令只接受 o / p")
        if action == last_action:
            continue
        result = gripper.set_open(action == "o")
        if not result["completion_confirmed"]:
            raise RuntimeError(result.get("note", "未确认夹爪运动完成"))
        last_action = action
```

调用会阻塞等待夹爪反馈；默认完整开合实测约 3 秒，不适合在机械臂的高频控制线程中直接等待。
接入需要并行运行的机械臂时，由独立的夹爪控制线程串行处理状态变化。
demo 未确认动作完成会以状态码 `2` 退出，不继续执行后续动作；无动作反馈不等于已经达到期望开口。
配置、方向和行程沿用 `config.json`。

## 接入 Python 程序

在仓库目录运行，或将该目录加入 Python 模块搜索路径。保持一个连接，按实际收到的业务指令调用：

```python
from dataclasses import asdict
from gripper import connect

with connect() as gripper:
    print(asdict(gripper.status()))

    result = gripper.command("close")  # 完整闭合
    if not result["completion_confirmed"]:
        raise RuntimeError(result.get("note", "未确认运动完成"))

    result = gripper.command("open")   # 完整张开
    print(result)
```

其他可用接口：

| API | 含义 |
|---|---|
| `gripper.set_open(True)` | 完整张开 |
| `gripper.set_open(False)` | 完整闭合 |
| `gripper.open()` / `gripper.close()` | 完整开合的简写 |
| `gripper.move("open", 180, 3)` | 指定方向、角度和速度点动 |
| `gripper.status()` | 返回包含角度、速度、到位标志的 `State` |
| `connect("other-config.json")` | 使用指定配置建立连接 |

运动方法阻塞等待反馈后返回字典。`completion_confirmed`、`motion_observed`、`outcome`
可用于业务判断；运动指令不会因反馈失败而自动重发。控制器到位只表示电机反馈，不能证明夹紧物体或夹持力。
本工具没有力控、爪口毫米定位或 TCP/HTTP 服务；业务程序需自行接收开关指令并调用这些 Python 方法。
每个串口使用一个程序、一个串行调用线程。

## 配置说明

完整模板见 [config.example.json](config.example.json)。

| 配置项 | 默认值 | 含义 |
|---|---|---|
| `port` | `auto` | 按 USB 序列号匹配；也可填写串口路径 |
| `usb_serial_number` | 空字符串 | 需填写自己的设备序列号；使用显式路径时忽略 |
| `baudrate` | `115200` | USB 串口波特率 |
| `device_id` | `1` | 控制协议设备地址 |
| `subdivision` | `32` | 电机细分值，支持 2、4、8、16、32 |
| `open_direction` | `0` | 张开方向位；闭合使用相反值 |
| `default_speed_rad_s` | `1.0` | 指定角度点动时的默认速度 |
| `response_timeout_s` | `2.0` | 常规查询反馈超时；键盘模式使用更短的查询超时 |
| `startup_delay_s` | `2.0` | 打开串口后的等待时间 |
| `full_travel_deg` | `1872.0` | 完整开合采用的电机行程 |
| `switch_speed_rad_s` | `10.0` | 完整开合速度 |
| `keyboard_step_deg` | `180.0` | 键盘模式保留的电机行程 |
| `keyboard_speed_rad_s` | `10.0` | 键盘模式速度 |

角度和速度最多一位小数。普通运动报文支持角度 0.1–6553.5°、速度 0.1–42 rad/s；
这些是程序接受的协议参数范围，不代表每一种夹爪机械结构都适合相应行程或速度。

## 调试与常见问题

```bash
bash gripper.sh --help
bash gripper.sh --verbose console --json
bash gripper.sh --output session.log console
bash gripper.sh --port /dev/ttyACM0 status --samples 3
```

全局参数如 `--config`、`--port`、`--output`、`--dry-run`、`--verbose` 放在子命令前。
普通 `console` 会打印简洁反馈，`--json` 显示完整结果。

| 现象 | 检查方式 |
|---|---|
| 找不到 `config.json` | 执行 `cp config.example.json config.json`，再填写设备信息 |
| 找不到串口或序列号不唯一 | 运行 `ports`，核对序列号或指定稳定串口路径 |
| 串口权限不足 | 检查设备所属组；Ubuntu 通常为 `dialout`，将用户加入对应组后重新登录 |
| 串口被占用 | 退出旧 console、键盘控制或其他串口程序 |
| 没有有效的 9 字节反馈 | 检查独立供电、115200 波特率和固件反馈支持；厂家更新记录注明 2026-03-20 恢复了数据反馈 |
| 小角度命令看起来不动 | 角度是电机角度；结合反馈判断，也可能已到该方向端点 |
| console 无动作但等待 | 连续反馈静止且角度未变化时，约 1 秒返回；未收到反馈则按查询超时处理 |
| 速度显示 0，但角度在变 | 被测设备闭合时出现过该现象；结合角度变化、到位标志判断，不能只看速度 |
| 键盘不响应 | 确认本地 X11 桌面、`DISPLAY` 和访问权限；不要依赖 SSH 字符输入 |
| `venv` 创建失败 | Ubuntu 可安装 `python3-venv`；若依赖需要编译，也需对应 Python 头文件和构建工具 |

## 协议摘要

实现依据为随附《睿尔曼夹爪使用手册》（2025-08-14 文件版本）的 USB 控制、数据反馈章节，
以及实物反馈验证。仓库不包含厂商手册、安装包或固件。

状态请求，设备地址 1：

```text
7b 01 00 00 00 00 00 00 00 7a 7d
```

USB 定位控制帧共 11 字节：

```text
7B ID 02 DIR SUB ANG_H ANG_L SPD_H SPD_L BCC 7D
```

- `DIR`：由 `open_direction` 映射；被测设备 0 张开、1 闭合。
- `ANG`：电机角度 ×10；`SPD`：rad/s ×10，均为高字节在前。
- `BCC`：前 9 字节逐字节异或。
- 手册总表的模式 `01` 与正文冲突，采用正文和开合示例一致、且已验证的 `02`。

设备地址 1、细分 32、张开 180°、速度 3 rad/s 的报文：

```text
7b 01 02 00 20 07 08 00 1e 49 7d
```

反馈共 9 字节：

```text
ID FLAG SPD_H SPD_L ANG_3 ANG_2 ANG_1 ANG_0 BCC
```

`FLAG` 为 0/1，速度和角度都除以 10；反馈校验为前 8 字节异或。
角度按**有符号 32 位数**解析，因为实物返回过负数补码；反馈解析支持分片、连续帧及校验失败后重新同步。

## 验证

2026-09-21 的实物验证，使用默认方向与速度配置：

| 项目 | 观察结果 |
|---|---|
| 无参数 `close` | 电机 19.2° → 1762.6°，约 3.28 秒到位 |
| 无参数 `open` | 电机 1762.6° → 21.0°，约 3.29 秒到位 |
| 完整开合目视 | 现场确认完整闭合后又张开 |
| 连续闭合 | 一次长按事件，6 次定位报文，电机 19.0° → 647.9° |
| 连续张开 | 一次长按事件，5 次定位报文，电机 647.9° → 109.0° |
| 松键后到位反馈 | 两次分别约 0.215 秒、0.409 秒 |

连续控制硬件测试向正式循环注入按下/松开回调事件；真实 pynput 按键事件另在隔离 Xvfb 桌面验证，
本地主机桌面的监听启动也已验证。上述记录用于说明已测范围，不代表所有固件和机械结构的保证值。

运行不连接硬件的测试：

```bash
.venv/bin/python -m unittest discover -v
```

30 项离线测试覆盖协议、方向映射、开关 API、`o/p` 按键与 `c` 不干扰、X11 重复事件处理、提前续发、
反向等待、端点无动作处理、超时、终端恢复和 VLA demo。另 1 项 pynput 事件集成测试默认跳过。
如已安装 `xvfb` 和 `xauth`，可在隔离桌面运行：

```bash
xvfb-run -a sh -c 'GRIPPER_TEST_DISPLAY="$DISPLAY" .venv/bin/python -m unittest test_keyboard_pynput -v'
```

集成测试会注入按键，`GRIPPER_TEST_DISPLAY` 只应指向隔离测试桌面。

## 文件结构

```text
gripper.py                 USB 协议、串口、Python API 和命令行入口
demo_vla.py                最简 VLA o/p 开关 demo，复用连接并跳过重复状态
keyboard_control.py        pynput 按键监听与连续控制循环
gripper.sh                 启动脚本
config.example.json        配置模板；复制为本地 config.json
requirements.txt           固定版本的 Python 依赖
test_gripper.py             协议与 console 测试
test_demo_vla.py            VLA demo 动作映射、连接复用与失败退出测试
test_keyboard_control.py    控制状态与开关 API 测试
test_keyboard_terminal.py   控制循环与终端恢复测试
test_keyboard_pynput.py     隔离桌面的真实 pynput 事件测试
```
