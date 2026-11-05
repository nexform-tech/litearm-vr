"""日志：转发路径上不打日志，这里只服务旁路（启动、告警、动作回执）。"""

from __future__ import annotations

import json
import logging
import sys
from datetime import datetime, timezone
from pathlib import Path

__all__ = ["setup_logging"]


class _JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload = {
            "ts": datetime.fromtimestamp(record.created, tz=timezone.utc).isoformat(
                timespec="milliseconds"
            ),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False)


class _TextFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        stamp = datetime.fromtimestamp(record.created, tz=timezone.utc).astimezone()
        base = f"{stamp:%H:%M:%S}.{int(stamp.microsecond // 1000):03d} "
        base += f"{record.levelname:<5} {record.name:<28} {record.getMessage()}"
        if record.exc_info:
            base += "\n" + self.formatException(record.exc_info)
        return base


def setup_logging(level: str = "INFO", fmt: str = "text", log_file: Path | None = None) -> None:
    root = logging.getLogger()
    for handler in list(root.handlers):
        root.removeHandler(handler)
        handler.close()

    formatter: logging.Formatter = (
        _JsonFormatter() if fmt.strip().lower() == "json" else _TextFormatter()
    )

    stream = logging.StreamHandler(sys.stdout)
    stream.setFormatter(formatter)
    handlers: list[logging.Handler] = [stream]

    if log_file is not None:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(log_file, encoding="utf-8")
        file_handler.setFormatter(formatter)
        handlers.append(file_handler)

    root.setLevel(getattr(logging, level.strip().upper(), logging.INFO))
    for handler in handlers:
        root.addHandler(handler)

    # 上游库里爱用 print/警告，压掉噪声，避免和我们的旁路日志抢 stdout
    logging.getLogger("websockets").setLevel(logging.WARNING)
