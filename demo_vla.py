"""最简 VLA 夹爪 demo：o 张开，p 闭合。"""

import argparse

from gripper import connect


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("actions", nargs="+", choices=("o", "p"),
                        help="o 张开、p 闭合；例如 p，或 o p o")
    args = parser.parse_args(argv)

    # 真实 VLA 接入时，将模型的夹爪动作映射成 o/p 指令流。
    # 在循环外连接一次，避免每次动作都重新连接串口。
    last_open = None
    with connect() as gripper:
        for action in args.actions:
            is_open = action == "o"
            if is_open == last_open:
                continue

            result = gripper.set_open(is_open)  # 一句控制完整开合，阻塞等待反馈。
            print(f"{action}: {result['outcome']}")
            if not result["completion_confirmed"]:
                print(result.get("note", "未确认动作完成，结束 demo。"))
                return 2
            last_open = is_open
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
