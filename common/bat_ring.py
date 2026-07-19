"""Python mirror of common/bat_ring.h — the shared-memory frame ring.

See bat_ring.h for the authoritative format description (64-byte ring header,
4 slots of [32-byte slot header + payload], seqlock per slot, latest-frame-
wins). This module must stay byte-for-byte in sync with the C header; the
interop test in tests/ checks that a C-written ring reads back here.

On QNX a POSIX shared-memory object named "/bat_cam0" appears as the file
/dev/shmem/bat_cam0, so both reading and creating rings works with ordinary
open()+mmap. On the Mac, plain file paths give the same format for host tests.

Stdlib-only (struct + mmap) so it runs anywhere, including the Pi.

Caveat: Python cannot issue CPU memory fences, so the *writer* here is not as
bullet-proof against instruction reordering as the C writer. In the worst case
a reader sees one torn frame (a garbled depth map for ~one update); acceptable
for now, and the camera-frame writer (the hot, contended path) is the C daemon.
"""

import mmap
import os
import struct
import time
from collections import namedtuple

MAGIC = 0x52544142  # "BATR"
VERSION = 1
NSLOTS = 4
NO_FRAME = 0xFFFFFFFF

FMT_NV12 = 1  # Y plane (w*h) then interleaved UV plane (w*h/2), stride == w
FMT_F32 = 2   # w*h little-endian float32 (depth map)

_HDR = struct.Struct("<8IQ")   # magic..latest (8 u32), wr_count (u64)
_HDR_SIZE = 64
_SLOT = struct.Struct("<IIQQQ")  # seq, size, t_capture, t_publish, frame_idx
_SLOT_SIZE = 32

Header = namedtuple(
    "Header", "magic version nslots format width height slot_size latest wr_count")
Meta = namedtuple("Meta", "size t_capture t_publish frame_idx")


def now_ns():
    """CLOCK_MONOTONIC in ns — same clock the C side stamps frames with."""
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def slot_stride(slot_size):
    return (_SLOT_SIZE + slot_size + 63) & ~63


def map_size(slot_size):
    return _HDR_SIZE + NSLOTS * slot_stride(slot_size)


def resolve_ring_path(name):
    """Turn a ring name or path into a mappable file path.

    Accepts a plain path ("./cam0.ring"), a shm name ("/bat_cam0" or
    "bat_cam0" — resolved under /dev/shmem on QNX), and returns the first
    that exists.
    """
    candidates = [name, "/dev/shmem/" + name.lstrip("/")]
    for path in candidates:
        if os.path.exists(path):
            return path
    raise FileNotFoundError(
        "ring %r not found (tried %s) — is the writer running?"
        % (name, ", ".join(candidates)))


class RingReader:
    """Read-only view of a ring. Poll read_latest(); it never blocks."""

    def __init__(self, path):
        self.path = path
        self._fd = os.open(path, os.O_RDONLY)
        size = os.fstat(self._fd).st_size
        if size < _HDR_SIZE:
            os.close(self._fd)
            raise ValueError("%s: too small to be a ring" % path)
        self._map = mmap.mmap(self._fd, size, prot=mmap.PROT_READ)
        hdr = self.header()  # validates magic/version
        if map_size(hdr.slot_size) > size:
            raise ValueError("%s: file smaller than its declared ring" % path)

    def header(self):
        hdr = Header(*_HDR.unpack_from(self._map, 0))
        if hdr.magic != MAGIC or hdr.version != VERSION or hdr.nslots != NSLOTS:
            raise ValueError("%s: not a bat_ring (or unsupported version)" % self.path)
        return hdr

    def read_latest(self, last_frame_idx=None):
        """Return (Meta, payload bytes) for the newest frame, or None if the
        ring is empty, the newest frame is still `last_frame_idx`, or every
        attempt raced the writer (just poll again)."""
        for _ in range(8):
            try:
                hdr = self.header()
            except ValueError:
                return None  # writer is re-initializing the ring right now
            idx = hdr.latest
            if idx == NO_FRAME or idx >= NSLOTS:
                return None
            off = _HDR_SIZE + idx * slot_stride(hdr.slot_size)
            seq0, size, t_cap, t_pub, frame_idx = _SLOT.unpack_from(self._map, off)
            if seq0 & 1:
                continue  # writer mid-copy
            if frame_idx == last_frame_idx:
                return None
            if size > hdr.slot_size:
                return None
            payload = bytes(self._map[off + _SLOT_SIZE:off + _SLOT_SIZE + size])
            seq1 = _SLOT.unpack_from(self._map, off)[0]
            if seq0 == seq1:
                return Meta(size, t_cap, t_pub, frame_idx), payload
        return None

    def close(self):
        self._map.close()
        os.close(self._fd)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class RingWriter:
    """Create (or re-initialize) a ring as its single writer."""

    def __init__(self, path, fmt, width, height, slot_size):
        self.path = path
        self._stride = slot_stride(slot_size)
        self._slot_size = slot_size
        self._wr_count = 0
        size = map_size(slot_size)
        self._fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
        os.ftruncate(self._fd, size)
        self._map = mmap.mmap(self._fd, size)
        self._map[:size] = b"\x00" * size
        _HDR.pack_into(self._map, 0, MAGIC, VERSION, NSLOTS, fmt,
                       width, height, slot_size, NO_FRAME, 0)

    def write(self, payload, t_capture=None, frame_idx=None):
        """Publish one frame. `payload` is bytes-like (e.g. ndarray.tobytes());
        `t_capture` propagates the source frame's capture timestamp (defaults
        to now); `frame_idx` defaults to this writer's own counter."""
        payload = bytes(payload)
        if len(payload) > self._slot_size:
            raise ValueError("payload %d > slot_size %d" % (len(payload), self._slot_size))
        idx = self._wr_count
        if frame_idx is None:
            frame_idx = idx
        if t_capture is None:
            t_capture = now_ns()
        s = idx % NSLOTS
        off = _HDR_SIZE + s * self._stride
        seq = _SLOT.unpack_from(self._map, off)[0]
        _SLOT.pack_into(self._map, off, seq + 1, 0, 0, 0, 0)  # odd: busy
        self._map[off + _SLOT_SIZE:off + _SLOT_SIZE + len(payload)] = payload
        _SLOT.pack_into(self._map, off, seq + 2, len(payload),
                        t_capture, now_ns(), frame_idx)  # even: stable
        self._wr_count = idx + 1
        # header: latest = s, wr_count += 1 (fields 7 and 8)
        struct.pack_into("<I", self._map, 7 * 4, s)
        struct.pack_into("<Q", self._map, 8 * 4, self._wr_count)

    def close(self):
        self._map.close()
        os.close(self._fd)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
