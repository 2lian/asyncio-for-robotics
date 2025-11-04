import json
import copy
import logging.config
import os
from datetime import datetime
from typing import Optional

import colorama
from colorama import Fore, Style, init


class JsonLineFormatter(logging.Formatter):
    """Outputs log records as JSON lines."""

    def format(self, record):
        log_record = {
            "time": datetime.fromtimestamp(record.created).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "funcName": record.funcName,
            "line": record.lineno,
        }
        if record.exc_info:
            log_record["exc_info"] = self.formatException(record.exc_info)
        return json.dumps(log_record)


LEVEL_COLORS = {
    "DEBUG": Fore.BLUE,
    "INFO": Fore.CYAN,
    "WARNING": Fore.YELLOW,
    "ERROR": Fore.RED,
    "CRITICAL": Fore.BLACK + Style.BRIGHT + colorama.Back.RED,
}


class OnlyLevelFilter(logging.Filter):
    """Only allow a single level through this handler."""
    def __init__(self, level):
        self.level = level
    def filter(self, record):
        return record.levelno == self.level

class ColoredFormatter(logging.Formatter):
    """Adds colors to levelname in logs."""

    def format(self, record: logging.LogRecord):
        record = copy.deepcopy(record)
        levelname = record.levelname
        if levelname in LEVEL_COLORS:
            record.levelname = (
                f"{LEVEL_COLORS[levelname]}{record.levelname[0]}{Style.RESET_ALL}"
            )
            record.msg = f"{LEVEL_COLORS[levelname]}{record.msg}{Style.RESET_ALL}"
        return super().format(record)


def setup_logger(debug_path: Optional[str] = None):
    handlers = ["stdout", "stderr"]
    if debug_path:
        handlers.append("json")
        handlers.append("userlog")
    cfg = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "user": {
                "()": ColoredFormatter,
                "format": "%(levelname)s| %(message)s",
                "datefmt": "%Y-%m-%dT%H:%M:%S",
            },
            "json": {
                "()": JsonLineFormatter,  # custom formatter
            },
        },
        "filters": {"only_info": {"()": OnlyLevelFilter, "level": logging.INFO}},
        "handlers": {
            "stdout": {
                "class": "logging.StreamHandler",
                "level": "INFO",
                "formatter": "user",
                "filters": ["only_info"],
                "stream": "ext://sys.stdout",
            },
            "stderr": {
                "class": "logging.StreamHandler",
                "level": "WARNING",
                "formatter": "user",
                "stream": "ext://sys.stderr",
            },
            "json": {
                "class": "logging.FileHandler",
                "level": "DEBUG",
                "formatter": "json",
                "filename": (
                    os.path.expanduser(debug_path) + "/debug.log.jsonl"
                    if debug_path is not None
                    else "log.jsonl"
                ),  # path relative to working dir
                "mode": "w",
            },
            "userlog": {
                "class": "logging.FileHandler",
                "level": "DEBUG",
                "formatter": "user",
                "filename": (
                    os.path.expanduser(debug_path) + "/debug.log"
                    if debug_path is not None
                    else "log"
                ),  # path relative to working dir
                "mode": "w",
            },
        },
        "loggers": {
            "asyncio_for_robotics": {
                "level": "DEBUG",
                "handlers": handlers,
                "propagate": False,
            },
        },
    }
    logging.config.dictConfig(cfg)

