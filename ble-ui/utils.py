from __future__ import annotations

import collections
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Deque, Optional


def current_time() -> datetime:
    """Return current UTC time as a timezone-aware datetime."""
    return datetime.now(timezone.utc)


def format_timestamp(dt: Optional[datetime]) -> str:
    """Format a datetime for display."""
    if dt is None:
        return "-"
    # Render in local time for user friendliness
    local_dt = dt.astimezone()
    return local_dt.strftime("%Y-%m-%d %H:%M:%S")


def age_seconds(dt: Optional[datetime]) -> Optional[float]:
    """Return age in seconds since dt, or None if dt is None."""
    if dt is None:
        return None
    return (current_time() - dt).total_seconds()


def bytes_to_hex(data: bytes) -> str:
    """Return a space-separated hex string for debug logging."""
    return " ".join(f"{b:02X}" for b in data)


def format_log_line(message: str, *, dt: Optional[datetime] = None) -> str:
    """Prefix log message with timestamp."""
    if dt is None:
        dt = current_time()
    ts = format_timestamp(dt)
    return f"[{ts}] {message}"


@dataclass
class LogBuffer:
    """Simple bounded log buffer for the UI log panel."""

    max_lines: int = 1000
    _lines: Deque[str] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self._lines is None:
            self._lines = collections.deque(maxlen=self.max_lines)

    def append(self, line: str) -> None:
        self._lines.append(line.rstrip("\n"))

    def clear(self) -> None:
        self._lines.clear()

    def get_lines(self) -> list[str]:
        return list(self._lines)

    def get_text(self) -> str:
        return "\n".join(self._lines)


