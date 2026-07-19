// Host unit test for the bat_ring shared-memory frame ring (file-backed, so
// no QNX or shared memory needed). Build and run on your dev machine with
//   make -C tests
// Usage: test_ring [ring-file-path]
// The ring file is left behind on success so test_ring_read.py can verify the
// Python mirror reads what the C writer wrote.
#include "bat_ring.h"

#include <cstdint>
#include <cstdio>
#include <vector>

static int failures = 0;

#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__);       \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

static uint8_t pattern_byte(int frame, size_t offset) {
    return static_cast<uint8_t>((frame * 31 + static_cast<int>(offset)) & 0xFF);
}

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "test_ring.ring";
    const uint32_t W = 8, H = 4;
    const uint32_t SLOT = W * H * 3 / 2;  // NV12 payload bytes

    bat_ring wr;
    CHECK(bat_ring_create(&wr, path, /*use_shm=*/0, BAT_FMT_NV12, W, H, SLOT) == 0,
          "create file-backed ring");
    CHECK(wr.hdr->magic == BAT_RING_MAGIC, "magic set");
    CHECK(wr.hdr->latest == BAT_RING_NO_FRAME, "no frame yet");

    std::vector<uint8_t> buf(SLOT);
    bat_slot_hdr meta;

    CHECK(bat_ring_read_latest(&wr, buf.data(), buf.size(), &meta) == 0,
          "read on empty ring reports no frame");

    // Write 10 frames (wrapping the 4 slots twice); verify after each that a
    // reader sees exactly the newest one.
    for (int i = 0; i < 10; ++i) {
        uint8_t* dst = bat_ring_write_begin(&wr);
        for (size_t j = 0; j < SLOT; ++j) dst[j] = pattern_byte(i, j);
        bat_ring_write_end(&wr, SLOT, /*t_capture=*/1000 + i);

        CHECK(bat_ring_read_latest(&wr, buf.data(), buf.size(), &meta) == 1,
              "read after write succeeds");
        CHECK(meta.frame_idx == static_cast<uint64_t>(i), "newest frame_idx wins");
        CHECK(meta.size == SLOT, "payload size");
        CHECK(meta.t_capture == static_cast<uint64_t>(1000 + i), "t_capture carried");
        CHECK(meta.t_publish > 0, "t_publish stamped");
        bool ok = true;
        for (size_t j = 0; j < SLOT; ++j)
            if (buf[j] != pattern_byte(i, j)) ok = false;
        CHECK(ok, "payload round-trips exactly");
    }
    CHECK(wr.hdr->wr_count == 10, "wr_count counts all writes");

    // Tear detection: fake a writer stuck mid-copy (odd seq) on the latest
    // slot; the reader must refuse rather than hand back a torn frame.
    bat_slot_hdr* latest_slot = bat_ring_slot(&wr, wr.hdr->latest);
    latest_slot->seq += 1;  // odd = busy
    CHECK(bat_ring_read_latest(&wr, buf.data(), buf.size(), &meta) == -1,
          "read refuses a busy slot");
    latest_slot->seq += 1;  // even again
    CHECK(bat_ring_read_latest(&wr, buf.data(), buf.size(), &meta) == 1,
          "read recovers once slot is stable");

    // A too-small reader buffer must be rejected, not overflowed.
    CHECK(bat_ring_read_latest(&wr, buf.data(), SLOT - 1, &meta) == -1,
          "read rejects undersized buffer");

    // A second process's view: open the same file read-only and read.
    bat_ring rd;
    CHECK(bat_ring_open(&rd, path, /*use_shm=*/0) == 0, "reader open");
    CHECK(rd.hdr->format == BAT_FMT_NV12 && rd.hdr->width == W && rd.hdr->height == H,
          "reader sees the writer's geometry");
    CHECK(bat_ring_read_latest(&rd, buf.data(), buf.size(), &meta) == 1, "reader reads");
    CHECK(meta.frame_idx == 9, "reader sees newest frame");
    bat_ring_close(&rd);

    // Opening a non-ring file must fail cleanly.
    bat_ring bad;
    CHECK(bat_ring_open(&bad, argv[0], /*use_shm=*/0) == -1,
          "open rejects a file that is not a ring");

    bat_ring_close(&wr);

    if (failures == 0) {
        std::printf("test_ring: all checks passed (ring left at %s)\n", path);
        return 0;
    }
    std::printf("test_ring: %d check(s) FAILED\n", failures);
    return 1;
}
