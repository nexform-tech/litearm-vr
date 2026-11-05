"""配置文件读取（`config/teleop.yaml`）。

只依赖标准库，**刻意如此**：`launch/run_teleop_webxr.py` 要在 metavr-arm 环境
建起来之前、用系统 python3 跑，它不能 import pybullet/numpy，也不该依赖第三方
YAML 库。所以这里自带一个 YAML **子集**解析器，够读本项目的配置文件即可。

支持的 YAML 子集（`config/teleop.yaml` 只用这些）：

  key: 值                 标量（int / float / true / false / null / ~ / 字符串）
  key: [a, b, c]          流式序列，可嵌套（3 维向量、3×3 矩阵）
  key:                    `key:` 后留空，下一行开始按缩进读嵌套映射
    子键: 值              嵌套用空格缩进（本项目统一 2 空格）
  # 注释                  行首或空白后的 `#` 起注释；引号内的 `#` 不算

**不支持**（遇到会报错，而不是默默读错）：块序列 `- item`、锚点/别名、
多行标量、文档分隔符 `---`、tab 缩进。十六进制之类写引号字符串（`"0x08"`）。

用法：
    from litearm_teleop.config import load, get
    cfg = load()                       # 出厂路径；传路径则读那个
    host = get(cfg, "server.host", "0.0.0.0")
"""
import os

# 出厂配置路径：<项目根>/config/teleop.yaml。本文件在
# <项目根>/src/litearm_teleop/ 下，往上 3 层到项目根。
_PROJECT_DIR = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
DEFAULT_PATH = os.path.join(_PROJECT_DIR, "config", "teleop.yaml")


def _strip_comment(line):
    """去掉行尾注释，但跳过引号内的 `#`。"""
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
        # 只认「行首或空白后」的 `#`，这样 URL 片段 / `a#b` 不会被腰斩
        if ch == "#" and (i == 0 or line[i - 1] in " \t"):
            break
        out.append(ch)
    return "".join(out).rstrip()


def _split_top(body):
    """按**顶层**逗号切分（方括号内的逗号不算），引号内的逗号也不算。"""
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
    """一个标量/流式序列字面量 → Python 值。"""
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
    """拆成 (缩进, 内容)，丢掉空行与注释行；顺便挡掉不支持的写法。"""
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
    """读一个缩进块的键值对，返回 (dict, 下一条下标)。"""
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
        # `key:` 后面留空：下一行缩进更深就是嵌套块，否则是空值
        if i + 1 < len(lines) and lines[i + 1][0] > indent:
            value, i = _parse_block(lines, i + 1, lines[i + 1][0])
            result[key] = value
        else:
            result[key] = None
            i += 1
    return result, i


def parse(text):
    """解析 YAML 子集文本 → 嵌套 dict。"""
    lines = _lines(text)
    if not lines:
        return {}
    if lines[0][0] != 0:
        raise ValueError("第 %d 行不应缩进（顶层键必须顶格）" % lines[0][2])
    data, _ = _parse_block(lines, 0, 0)
    return data


def load(path=None):
    """读配置文件 → 嵌套 dict。

    path 为 None 时读出厂路径 `config/teleop.yaml`。文件不存在由调用方决定怎么
    处理（启动脚本会警告并退回出厂常量），所以这里不吞 IOError。
    """
    with open(DEFAULT_PATH if path is None else path, "r", encoding="utf-8") as f:
        return parse(f.read())


def get(data, path, default=None):
    """按 `"a.b.c"` 取嵌套值；中间任一层缺失就返回 default。

    调用方传的 default 就是该选项的**出厂常量** —— 所以配置文件里少写一个键
    不会改变行为，只会少一个可调点。显式写成 null 的键同样返回 default（null
    在本项目里的语义就是「用出厂值」）。
    """
    node = data
    for key in path.split("."):
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return default if node is None else node


def set_path(data, path, value):
    """按 `"a.b.c"` 写嵌套值（中间层不存在就建）。命令行覆盖配置文件时用。"""
    keys = path.split(".")
    node = data
    for key in keys[:-1]:
        child = node.get(key)
        if not isinstance(child, dict):
            child = {}
            node[key] = child
        node = child
    node[keys[-1]] = value
    return data
