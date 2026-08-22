"""Small bounded exact-stamp caches for asynchronous ROS sensor callbacks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CacheEntry:
    stamp_ns: int
    value: Any
    received_ns: int


class BoundedStampCache:
    def __init__(self, capacity: int, max_age_ns: int) -> None:
        self.capacity = max(1, int(capacity))
        self.max_age_ns = max(1, int(max_age_ns))
        self._entries: dict[int, CacheEntry] = {}

    def put(self, stamp_ns: int, value: Any, received_ns: int) -> None:
        self.expire(received_ns)
        self._entries[int(stamp_ns)] = CacheEntry(int(stamp_ns), value, int(received_ns))
        while len(self._entries) > self.capacity:
            del self._entries[min(self._entries)]

    def pop(self, stamp_ns: int, now_ns: int) -> Any | None:
        self.expire(now_ns)
        entry = self._entries.pop(int(stamp_ns), None)
        return None if entry is None else entry.value

    def pop_oldest(self, now_ns: int) -> tuple[int, Any] | None:
        self.expire(now_ns)
        if not self._entries:
            return None
        stamp = min(self._entries)
        return stamp, self._entries.pop(stamp).value

    def pop_newest(self, now_ns: int) -> tuple[int, Any] | None:
        self.expire(now_ns)
        if not self._entries:
            return None
        stamp = max(self._entries)
        return stamp, self._entries.pop(stamp).value

    def expire(self, now_ns: int) -> None:
        cutoff = int(now_ns) - self.max_age_ns
        for stamp, entry in list(self._entries.items()):
            if entry.received_ns < cutoff:
                del self._entries[stamp]

    def __len__(self) -> int:
        return len(self._entries)


class ExactStampPairCache:
    """Pair only identical message stamps; no approximate-time fallback exists."""

    def __init__(self, capacity: int, max_age_ns: int) -> None:
        self._left = BoundedStampCache(capacity, max_age_ns)
        self._right = BoundedStampCache(capacity, max_age_ns)

    def add_left(self, stamp_ns: int, value: Any, now_ns: int) -> tuple[int, Any, Any] | None:
        return self._add(self._left, self._right, stamp_ns, value, now_ns, left_first=True)

    def add_right(self, stamp_ns: int, value: Any, now_ns: int) -> tuple[int, Any, Any] | None:
        return self._add(self._right, self._left, stamp_ns, value, now_ns, left_first=False)

    def _add(self, own: BoundedStampCache, other: BoundedStampCache, stamp_ns: int, value: Any, now_ns: int, *, left_first: bool) -> tuple[int, Any, Any] | None:
        matched = other.pop(stamp_ns, now_ns)
        if matched is None:
            own.put(stamp_ns, value, now_ns)
            return None
        return (int(stamp_ns), value, matched) if left_first else (int(stamp_ns), matched, value)

    def expire(self, now_ns: int) -> None:
        self._left.expire(now_ns)
        self._right.expire(now_ns)
