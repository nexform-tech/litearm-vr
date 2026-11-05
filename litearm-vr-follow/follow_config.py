#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""follow.yaml 读取 + 各 run_*.sh 的启动包装。

run_*.sh 里不再手写参数，而是：

    exec python3 follow_config.py --exec <段名> -- "$@"

本脚本读同目录的 follow.yaml，把该段落的键翻成命令行参数，再把命令行给的参数
追加在后（argparse 后者胜出）⇒ 实现 **命令行 > follow.yaml > 出厂常量**。
命令行里已出现的 flag，其 yaml 值会被整段跳过（否则 append 类参数会叠加）。

只依赖标准库，**刻意如此**：run_*.sh 要在各 SDK 备齐之前就能跑，不该依赖第三方
YAML 库。故自带一个 YAML 子集解析器（与 litearm-vr-teleop 的 config.py 同源）。

调试用法：
    python3 follow_config.py --print arm_sdk -- --tcp-port 9002   # 只打印将执行的命令
    python3 follow_config.py --help
"""
import os
import shlex
import sys

# 出厂配置路径：与本脚本同目录的 follow.yaml
DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "follow.yaml")

# ---------------------------------------------------------------------------
# 段名 → (服务脚本, [(yaml 键, 命令行 flag, 种类)])
#   种类：scalar=--f v | nargs=--f v1 v2 … | append=--f v（逐项重复）
#         | flag=store_true（真则出现，假/空则略过）| positional=裸位置参数
# ---------------------------------------------------------------------------
SCHEMA = {
    "arm_stm32": ("arm_follow_server_stm32.py", [
        ("port", "port", "scalar"),
        ("mode", "mode", "scalar"),
        ("host", "host", "scalar"),
        ("tcp_port", "tcp-port", "scalar"),
        ("kp", "kp", "nargs"),
        ("kd", "kd", "nargs"),
        ("sp", "sp", "scalar"),
        ("q_min", "q-min", "nargs"),
        ("q_max", "q-max", "nargs"),
        ("max_speed", "max-speed", "nargs"),
        ("max_accel", "max-accel", "nargs"),
    ]),
    "arm_sdk": ("arm_follow_server_sdk.py", [
        ("config", "config", "scalar"),
        ("iface", "iface", "scalar"),
        ("dry_run", "dry-run", "flag"),
        ("host", "host", "scalar"),
        ("tcp_port", "tcp-port", "scalar"),
        ("kp", "kp", "nargs"),
        ("kd", "kd", "nargs"),
        ("speed_limit", "speed-limit", "nargs"),
        ("accel_limit", "accel-limit", "nargs"),
        ("sp", "sp", "scalar"),
        ("engage_sec", "engage-sec", "scalar"),
    ]),
    "gripper": ("gripper_server.py", [
        ("can", "can", "scalar"),
        ("can_id", "can-id", "scalar"),
        ("mst_id", "mst-id", "scalar"),
        ("mount", "mount", "scalar"),
        ("calib", "calib", "scalar"),
        ("host", "host", "scalar"),
        ("port", "port", "scalar"),
        ("speed", "speed", "scalar"),
        ("force", "force", "scalar"),
        ("rate_hz", "rate-hz", "scalar"),
        ("kp", "kp", "scalar"),
        ("kd", "kd", "scalar"),
        ("max_pos_error_mm", "max-pos-error-mm", "scalar"),
        ("max_fb_vel", "max-fb-vel", "scalar"),
        ("feedback_timeout", "feedback-timeout", "scalar"),
        ("temp_limit", "temp-limit", "scalar"),
        ("status_hz", "status-hz", "scalar"),
        ("no_startup_disable", "no-startup-disable", "flag"),
        ("no_startup_open", "no-startup-open", "flag"),
    ]),
    "hand": ("hand_server.py", [
        ("can", "can", "scalar"),
        ("host", "host", "scalar"),
        ("port", "port", "scalar"),
    ]),
    "aggregator": ("status_aggregator.py", [
        ("arm", "arm", "append"),
        ("gripper", "gripper", "append"),
        ("hand", "hand", "append"),
        ("prefix", "prefix", "scalar"),
        ("arm_hz", "arm-hz", "scalar"),
        ("gripper_hz", "gripper-hz", "scalar"),
        ("publish_hz", "publish-hz", "scalar"),
        ("stale_ms", "stale-ms", "scalar"),
        ("connect_timeout_s", "connect-timeout-s", "scalar"),
        ("request_timeout_s", "request-timeout-s", "scalar"),
        ("zenoh_mode", "zenoh-mode", "scalar"),
        ("zenoh_listen", "zenoh-listen", "append"),
        ("zenoh_listen_port", "zenoh-listen-port", "scalar"),
        ("zenoh_connect", "zenoh-connect", "append"),
        ("zenoh_scouting", "zenoh-scouting", "flag"),
        ("zenoh_config", "zenoh-config", "scalar"),
        ("quiet", "quiet", "flag"),
        ("verbose", "verbose", "flag"),
    ]),
    "stm32_comm": ("test_stm32_comm.py", [
        ("port", "", "positional"),
    ]),
}

USAGE = """\
用法：follow_config.py [--exec|--print] [--config PATH] <段名> [-- 命令行参数…]

段名：%s

  --exec          读 follow.yaml 后 exec 服务脚本（默认行为，run_*.sh 用这个）
  --print         只打印将要执行的命令，不真的 exec（调试用）
  --config PATH   指定 follow.yaml（默认同目录 follow.yaml；也可用环境变量 FOLLOW_CONFIG）
  -- 之后        原样透传给服务脚本，覆盖 yaml 里的同名选项
""" % ", ".join(SCHEMA)


# ---------------------------------------------------------------------------
# YAML 子集解析器（与 litearm-vr-teleop 的 src/litearm_teleop/config.py 同源）
# 支持：key: 值 / key: [a, b] / key: 后缩进嵌套映射 / # 注释。只用标准库。
# 不支持：块序列 `- item`、锚点、多行标量、---、tab 缩进（遇到会报错而非读错）。
# ---------------------------------------------------------------------------
def _strip_comment(line):
    out = []
    quote = None
    for i, ch in enumerate(line):
        if quote:
            out.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in "\"'":
            quote = ch
            out.append(ch)
            continue
        if ch == "#" and (i == 0 or line[i - 1] in " \t"):
            break
        out.append(ch)
    return "".join(out).rstrip()


def _split_top(body):
    items = []
    cur = []
    depth = 0
    quote = None
    for ch in body:
        if quote:
            cur.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in "\"'":
            quote = ch
            cur.append(ch)
            continue
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth -= 1
        elif ch == "," and depth == 0:
            items.append("".join(cur).strip())
            cur = []
            continue
        cur.append(ch)
    items.append("".join(cur).strip())
    return [item for item in items if item != ""]


def _unquote(text):
    quote = text[0]
    if len(text) < 2 or text[-1] != quote:
        raise ValueError("引号没有配对: %r" % text)
    body = text[1:-1]
    return body.replace("\\" + quote, quote).replace("\\\\", "\\")


def _scalar(text):
    text = text.strip()
    if not text:
        return None
    if text[0] == "[":
        if text[-1] != "]":
            raise ValueError("序列必须以 ] 结尾: %r" % text)
        return [_scalar(item) for item in _split_top(text[1:-1])]
    if text[0] in "\"'":
        return _unquote(text)
    low = text.lower()
    if low in ("null", "~"):
        return None
    if low == "true":
        return True
    if low == "false":
        return False
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return text


def _lines(text):
    out = []
    for number, raw in enumerate(text.splitlines(), 1):
        if "\t" in raw[:len(raw) - len(raw.lstrip())]:
            raise ValueError("第 %d 行用了 tab 缩进，请改用空格: %r" % (number, raw))
        line = _strip_comment(raw)
        if not line.strip():
            continue
        if line.strip().startswith("---"):
            raise ValueError("第 %d 行：不支持文档分隔符 ---" % number)
        if line.lstrip().startswith("- "):
            raise ValueError("第 %d 行：不支持块序列（- item），请用 [a, b, c]" % number)
        indent = len(line) - len(line.lstrip())
        out.append((indent, line.strip(), number))
    return out


def _parse_block(lines, i, indent):
    result = {}
    while i < len(lines):
        col, text, number = lines[i]
        if col < indent:
            break
        if col > indent:
            raise ValueError("第 %d 行缩进突进（%d 空格），上一行没有以 `key:` 收尾"
                             % (number, col))
        key, sep, rest = text.partition(":")
        if not sep or not key.strip():
            raise ValueError("第 %d 行不是 `键: 值` 形式: %r" % (number, text))
        key = key.strip()
        rest = rest.strip()
        if rest:
            result[key] = _scalar(rest)
            i += 1
            continue
        if i + 1 < len(lines) and lines[i + 1][0] > indent:
            value, i = _parse_block(lines, i + 1, lines[i + 1][0])
            result[key] = value
        else:
            result[key] = None
            i += 1
    return result, i


def parse(text):
    lines = _lines(text)
    if not lines:
        return {}
    if lines[0][0] != 0:
        raise ValueError("第 %d 行不应缩进（顶层键必须顶格）" % lines[0][2])
    data, _ = _parse_block(lines, 0, 0)
    return data


def load(path=None):
    with open(DEFAULT_PATH if path is None else path, "r", encoding="utf-8") as f:
        return parse(f.read())


# ---------------------------------------------------------------------------
# 段 → 命令行参数
# ---------------------------------------------------------------------------
def _present_flags(user_args):
    """命令行里已出现的 flag 名（`--f` 与 `--f=v` 都算 `--f`）。"""
    found = set()
    for arg in user_args:
        if arg.startswith("--") and arg != "--":
            found.add(arg.split("=", 1)[0])
    return found


def build_argv(section, section_data, user_args):
    """把 yaml 段落翻成 argv 列表；命令行已给的 flag 让位（整段跳过）。"""
    present = _present_flags(user_args)
    out = []
    for key, flag, kind in SCHEMA[section][1]:
        if key not in section_data:
            continue                      # 用户删了键 → 回退服务脚本出厂值
        value = section_data[key]
        if kind == "flag":
            if value and ("--" + flag) not in present:
                out.append("--" + flag)
            continue
        if value is None:
            continue
        if kind == "positional":
            # 位置参数没有 flag 名可比，只要命令行给了任何参数就当覆盖
            # （stm32_comm 段的服务 test_stm32_comm.py 只吃位置参数）
            if user_args:
                continue
            out.append(str(value))
            continue
        if "--" + flag in present:
            continue
        if kind in ("nargs", "append") and not isinstance(value, list):
            value = [value]
        if kind == "append":
            for item in value:
                out += ["--" + flag, str(item)]
        elif kind == "nargs":
            out += ["--" + flag] + [str(item) for item in value]
        else:                             # scalar
            out += ["--" + flag, str(value)]
    return out


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def _split_section(argv):
    if "--" in argv:
        cut = argv.index("--")
        return argv[:cut], argv[cut + 1:]
    return argv, []


def main(argv):
    head, user_args = _split_section(argv)
    mode = "exec"
    path = os.environ.get("FOLLOW_CONFIG") or DEFAULT_PATH
    section = None
    i = 0
    while i < len(head):
        arg = head[i]
        if arg in ("-h", "--help"):
            sys.stdout.write(USAGE)
            return 0
        if arg == "--print":
            mode = "print"
            i += 1
            continue
        if arg == "--exec":
            mode = "exec"
            i += 1
            continue
        if arg == "--config":
            if i + 1 >= len(head):
                sys.stderr.write("错误：--config 缺参数\n")
                return 2
            path = head[i + 1]
            i += 2
            continue
        if arg.startswith("-"):
            sys.stderr.write("错误：未知选项 %s\n%s" % (arg, USAGE))
            return 2
        section = arg
        i += 1

    if section is None:
        sys.stderr.write("错误：必须给一个段名\n%s" % USAGE)
        return 2
    if section not in SCHEMA:
        sys.stderr.write("错误：未知段名 %r（可选：%s）\n"
                         % (section, ", ".join(SCHEMA)))
        return 2

    try:
        data = load(path)
    except FileNotFoundError:
        sys.stderr.write("[follow_config] 未找到 %s，本次全用服务脚本出厂值\n" % path)
        data = {}
    except ValueError as exc:
        sys.stderr.write("[follow_config] 解析 %s 失败：%s\n" % (path, exc))
        return 2

    section_data = data.get(section)
    if section_data is None:
        section_data = {}
    elif not isinstance(section_data, dict):
        sys.stderr.write("[follow_config] %s 里的 %r 段不是一个映射\n" % (path, section))
        return 2

    script = os.path.join(os.path.dirname(DEFAULT_PATH), SCHEMA[section][0])
    argv_out = [script] + build_argv(section, section_data, user_args) + user_args

    if mode == "print":
        print(shlex.join(["python3"] + argv_out))
        return 0

    os.execvp("python3", ["python3"] + argv_out)
    return 0  # 不会到这


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
