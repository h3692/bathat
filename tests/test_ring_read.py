#!/usr/bin/env python3
"""Interop check: read a ring file written by the C test (test_ring) with the
Python mirror (common/bat_ring.py) and verify the newest frame byte-for-byte.
Stdlib-only. Run by `make -C tests`; usage: test_ring_read.py <ring-file>
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "common"))
import bat_ring


def pattern_byte(frame, offset):
    return (frame * 31 + offset) & 0xFF


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: test_ring_read.py <ring-file>")

    with bat_ring.RingReader(sys.argv[1]) as reader:
        hdr = reader.header()
        assert hdr.format == bat_ring.FMT_NV12, hdr
        assert (hdr.width, hdr.height) == (8, 4), hdr
        assert hdr.slot_size == 8 * 4 * 3 // 2, hdr
        assert hdr.wr_count == 10, hdr

        got = reader.read_latest()
        assert got is not None, "no frame readable"
        meta, payload = got
        assert meta.frame_idx == 9, meta
        assert meta.t_capture == 1009, meta
        assert meta.size == len(payload) == hdr.slot_size, meta
        bad = [j for j in range(len(payload))
               if payload[j] != pattern_byte(9, j)]
        assert not bad, "payload mismatch at offsets %s..." % bad[:5]

        # De-duplication: the same frame must not be handed out twice.
        assert reader.read_latest(last_frame_idx=meta.frame_idx) is None

    print("test_ring_read: C-written ring reads back correctly in Python")


if __name__ == "__main__":
    main()
