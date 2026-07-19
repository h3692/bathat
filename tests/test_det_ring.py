#!/usr/bin/env python3
"""Round-trip check for the detections ring format (BAT_FMT_DET): pack a
detection list, publish it through RingWriter, read it back with RingReader,
and verify every field. Stdlib-only. Run by `make -C tests`; usage:
test_det_ring.py <ring-file>
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "common"))
import bat_ring


def close_to(a, b, tol=1e-6):
    return abs(a - b) <= tol


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: test_det_ring.py <ring-file>")
    path = sys.argv[1]

    # Layout constants: 8-byte header + 8 records of six float32.
    assert bat_ring.FMT_DET == 4
    assert bat_ring.DET_NMAX == 8
    assert bat_ring.DET_SLOT_SIZE == 8 + 8 * 24 == 200

    dets = [
        bat_ring.Detection(cx=0.25, cy=0.5, radius=0.05,
                           closeness=0.9, azimuth_deg=-30.0, area_frac=0.01),
        bat_ring.Detection(cx=0.75, cy=0.4, radius=0.1,
                           closeness=0.4, azimuth_deg=52.5, area_frac=0.03),
    ]

    with bat_ring.RingWriter(path, bat_ring.FMT_DET, 256, 256,
                             bat_ring.DET_SLOT_SIZE) as writer:
        writer.write(bat_ring.pack_detections(dets), t_capture=1234)

        with bat_ring.RingReader(path) as reader:
            hdr = reader.header()
            assert hdr.format == bat_ring.FMT_DET, hdr
            assert (hdr.width, hdr.height) == (256, 256), hdr
            assert hdr.slot_size == bat_ring.DET_SLOT_SIZE, hdr

            got = reader.read_latest()
            assert got is not None, "no frame readable"
            meta, payload = got
            assert meta.t_capture == 1234, meta
            assert meta.size == bat_ring.DET_SLOT_SIZE, meta

            back = bat_ring.unpack_detections(payload)
            assert len(back) == 2, back
            for want, have in zip(dets, back):
                for field in want._fields:
                    assert close_to(getattr(want, field), getattr(have, field)), \
                        (field, want, have)

        # Overfull lists are capped at DET_NMAX, empty lists round-trip.
        packed = bat_ring.pack_detections([dets[0]] * 12)
        assert len(packed) == bat_ring.DET_SLOT_SIZE
        assert len(bat_ring.unpack_detections(packed)) == bat_ring.DET_NMAX
        assert bat_ring.unpack_detections(bat_ring.pack_detections([])) == []

    print("test_det_ring: DET detections round-trip correctly")


if __name__ == "__main__":
    main()
