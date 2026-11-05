"""Zenoh 会话：本服务在 zenoh 网络上的唯一一条连接。

**一个进程一条会话** —— 帧发布、状态发布、动作 queryable 全挂在它上面。开两条会变成
两个连接、两份状态，还会让"对端看到几个订阅者"这类判断失真。

会话的拧法照抄 `litearm-server` 的 `ZenohTransport.__init__`（那是本产品既有的写法）：

* 程序化 `zenoh.Config()` + `insert_json5`，**不用配置文件** —— 配置跟代码走，现场少
  一个能改坏、又不在版本控制里的东西；
* `scouting/multicast` 与 `scouting/gossip` 显式关掉：拓扑是固定的（谁连谁写在部署
  里），用发现反而让"连不上"变成"连上了别的什么"；
* `mode` + 显式的 listen/connect 端点。本服务默认 **peer + listen**：它既主动连
  router，也让 teleop 直连进来（`ctl` 就是这么连的，见 ctl.py）。

`import zenoh` 放在函数里：`--doctor` 要在没装 zenoh 的机器上也把报告打出来
（那条检查自己会报 FAIL），不能让 import 把整个预检带崩。
"""

from __future__ import annotations

import logging
from typing import Any

from .config import Config

log = logging.getLogger("litearm_vr_bridge.session")

__all__ = ["open_session", "close_session", "describe"]


def open_session(cfg: Config) -> Any:
    """按配置开一条 zenoh 会话。调用方负责 `close_session`。"""
    import zenoh

    zcfg = zenoh.Config()
    zcfg.insert_json5("mode", f'"{cfg.zenoh_mode}"')
    zcfg.insert_json5("scouting/multicast/enabled", "false")
    zcfg.insert_json5("scouting/gossip/enabled", "false")
    # 查询默认超时（毫秒）。冷启动 teleop.start 要十几秒（等端口 + 等上游），
    # 超时按它定，否则会出现"动作其实成功了、上位机却报超时"。
    zcfg.insert_json5("queries_default_timeout", str(int(cfg.zenoh_reply_timeout_s * 1000)))
    if cfg.zenoh_listen_endpoints:
        zcfg.insert_json5("listen/endpoints", _json_array(cfg.zenoh_listen_endpoints))
    if cfg.zenoh_connect_endpoints:
        zcfg.insert_json5("connect/endpoints", _json_array(cfg.zenoh_connect_endpoints))

    session = zenoh.open(zcfg)
    log.info("zenoh 会话已开（%s）", describe(cfg))
    return session


def close_session(session: Any) -> None:
    """关会话；已经关过、或压根没开成，都不算错。"""
    if session is None:
        return
    try:
        session.close()
    except Exception as exc:  # noqa: BLE001
        log.debug("关 zenoh 会话时出错（忽略）：%s", exc)


def describe(cfg: Config) -> str:
    """一句话说清这条会话往哪连、听在哪 —— 日志和 `--doctor` 都用它。"""
    parts = [f"mode={cfg.zenoh_mode}", f"arm_id={cfg.zenoh_arm_id}"]
    if cfg.zenoh_listen_endpoints:
        parts.append("listen=" + ",".join(cfg.zenoh_listen_endpoints))
    if cfg.zenoh_connect_endpoints:
        parts.append("connect=" + ",".join(cfg.zenoh_connect_endpoints))
    else:
        parts.append("connect=(无，等对端连进来)")
    return " ".join(parts)


def _json_array(values: tuple[str, ...]) -> str:
    return "[" + ",".join(f'"{v}"' for v in values) + "]"
