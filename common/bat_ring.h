#ifndef BAT_RING_H
#define BAT_RING_H

/*
 * bat_ring — single-writer / multi-reader, latest-frame-wins shared-memory
 * frame ring. This is the hand-off point between pipeline stages that live in
 * different processes: the C capture daemon publishes camera frames into
 * "/bat_cam0" and "/bat_cam1", the Python depth worker reads those and
 * publishes depth maps into "/bat_depth0"/"/bat_depth1".
 *
 * "Latest-frame-wins" means the ring never blocks and never queues: a slow
 * reader simply misses frames (intentional — stale camera frames are useless).
 * A "seqlock" (sequence lock) makes that safe without OS locks: each slot has
 * a sequence counter that is odd while the writer is mid-copy and even when
 * the slot is stable. A reader copies the payload, then checks the counter is
 * even and unchanged; if not, the copy was torn and it retries.
 *
 * Memory layout (little-endian, fixed offsets — mirrored by common/bat_ring.py):
 *
 *   [bat_ring_hdr, 64 bytes]
 *   [slot 0][slot 1]...[slot BAT_RING_NSLOTS-1]
 *
 * each slot, stride padded to a 64-byte multiple:
 *   [bat_slot_hdr, 32 bytes][payload, slot_size bytes]
 *
 * Backing storage is POSIX shared memory (QNX exposes it at
 * /dev/shmem/<name>, so Python can mmap it by path), or a plain file for
 * host-side tests — the format is identical.
 *
 * Header-only; compiles as C11 and C++17. Single writer per ring assumed.
 */

#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BAT_RING_MAGIC 0x52544142u /* "BATR" little-endian */
#define BAT_RING_VERSION 1u
#define BAT_RING_NSLOTS 4u
#define BAT_RING_NO_FRAME 0xFFFFFFFFu

enum bat_ring_format {
    BAT_FMT_NV12 = 1, /* Y plane (width*height), then interleaved UV plane
                         (width*height/2); tight stride == width */
    BAT_FMT_F32 = 2,  /* width*height little-endian float32 (depth map) */
    BAT_FMT_BGR8 = 3, /* width*height*3 bytes, B,G,R interleaved (rectified colour) */
};

typedef struct bat_ring_hdr {
    uint32_t magic;     /* BAT_RING_MAGIC; written last during init */
    uint32_t version;   /* BAT_RING_VERSION */
    uint32_t nslots;    /* BAT_RING_NSLOTS */
    uint32_t format;    /* enum bat_ring_format */
    uint32_t width;     /* payload pixel dimensions */
    uint32_t height;
    uint32_t slot_size; /* payload capacity per slot, bytes */
    uint32_t latest;    /* slot index of newest complete frame, or BAT_RING_NO_FRAME */
    uint64_t wr_count;  /* total frames ever written */
    uint8_t pad[64 - 40];
} bat_ring_hdr;

typedef struct bat_slot_hdr {
    uint32_t seq;       /* seqlock counter: odd while the writer is copying */
    uint32_t size;      /* valid payload bytes (<= slot_size) */
    uint64_t t_capture; /* CLOCK_MONOTONIC ns when the source camera frame
                           arrived; propagated through the pipeline so
                           end-to-end latency stays measurable */
    uint64_t t_publish; /* CLOCK_MONOTONIC ns when this slot was written */
    uint64_t frame_idx; /* the writer's frame counter for this frame */
} bat_slot_hdr;

typedef char bat_ring_hdr_is_64_bytes[(sizeof(bat_ring_hdr) == 64) ? 1 : -1];
typedef char bat_slot_hdr_is_32_bytes[(sizeof(bat_slot_hdr) == 32) ? 1 : -1];

/* One process's view of a mapped ring. */
typedef struct bat_ring {
    bat_ring_hdr* hdr;
    uint8_t* base;
    size_t map_size;
    size_t slot_stride;
    int fd;
} bat_ring;

static inline uint64_t bat_ring_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline size_t bat_ring_slot_stride(uint32_t slot_size) {
    return ((size_t)sizeof(bat_slot_hdr) + slot_size + 63u) & ~(size_t)63u;
}

static inline size_t bat_ring_map_size(uint32_t slot_size) {
    return sizeof(bat_ring_hdr) + (size_t)BAT_RING_NSLOTS * bat_ring_slot_stride(slot_size);
}

static inline bat_slot_hdr* bat_ring_slot(const bat_ring* r, uint32_t i) {
    return (bat_slot_hdr*)(r->base + sizeof(bat_ring_hdr) + (size_t)i * r->slot_stride);
}

static inline uint8_t* bat_ring_payload(const bat_ring* r, uint32_t i) {
    return (uint8_t*)bat_ring_slot(r, i) + sizeof(bat_slot_hdr);
}

/* Create (or re-initialize) a ring as its single writer. `use_shm` selects
 * POSIX shared memory (name like "/bat_cam0") vs a plain file path (host
 * tests). Returns 0 on success, -1 with errno set on failure. */
static inline int bat_ring_create(bat_ring* r, const char* name, int use_shm,
                                  uint32_t format, uint32_t width, uint32_t height,
                                  uint32_t slot_size) {
    memset(r, 0, sizeof(*r));
    int fd = use_shm ? shm_open(name, O_CREAT | O_RDWR, 0666)
                     : open(name, O_CREAT | O_RDWR, 0666);
    if (fd < 0) return -1;

    const size_t sz = bat_ring_map_size(slot_size);
    if (ftruncate(fd, (off_t)sz) != 0) {
        close(fd);
        return -1;
    }
    void* p = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        close(fd);
        return -1;
    }

    r->base = (uint8_t*)p;
    r->map_size = sz;
    r->slot_stride = bat_ring_slot_stride(slot_size);
    r->fd = fd;
    r->hdr = (bat_ring_hdr*)p;

    /* Zero everything, fill the header, and only then set the magic (with a
     * release store) so a reader never sees a half-initialized ring. */
    memset(p, 0, sz);
    r->hdr->version = BAT_RING_VERSION;
    r->hdr->nslots = BAT_RING_NSLOTS;
    r->hdr->format = format;
    r->hdr->width = width;
    r->hdr->height = height;
    r->hdr->slot_size = slot_size;
    r->hdr->latest = BAT_RING_NO_FRAME;
    __atomic_store_n(&r->hdr->magic, BAT_RING_MAGIC, __ATOMIC_RELEASE);
    return 0;
}

/* Open an existing ring read-only. Returns 0 on success, -1 on failure
 * (errno set, or format mismatch). */
static inline int bat_ring_open(bat_ring* r, const char* name, int use_shm) {
    memset(r, 0, sizeof(*r));
    int fd = use_shm ? shm_open(name, O_RDONLY, 0) : open(name, O_RDONLY);
    if (fd < 0) return -1;

    struct stat st;
    if (fstat(fd, &st) != 0 || (size_t)st.st_size < sizeof(bat_ring_hdr)) {
        close(fd);
        return -1;
    }
    void* p = mmap(NULL, (size_t)st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        close(fd);
        return -1;
    }

    r->base = (uint8_t*)p;
    r->map_size = (size_t)st.st_size;
    r->fd = fd;
    r->hdr = (bat_ring_hdr*)p;

    if (__atomic_load_n(&r->hdr->magic, __ATOMIC_ACQUIRE) != BAT_RING_MAGIC ||
        r->hdr->version != BAT_RING_VERSION || r->hdr->nslots != BAT_RING_NSLOTS ||
        bat_ring_map_size(r->hdr->slot_size) > r->map_size) {
        munmap(p, r->map_size);
        close(fd);
        memset(r, 0, sizeof(*r));
        return -1;
    }
    r->slot_stride = bat_ring_slot_stride(r->hdr->slot_size);
    return 0;
}

/* Unmap. Never removes the shm object itself, so readers can attach at any
 * time and a restarted writer just re-initializes it. */
static inline void bat_ring_close(bat_ring* r) {
    if (r->base) munmap(r->base, r->map_size);
    if (r->fd >= 0) close(r->fd);
    memset(r, 0, sizeof(*r));
    r->fd = -1;
}

/* Writer: claim the next slot and return its payload pointer (capacity
 * hdr->slot_size). Fill it, then call bat_ring_write_end. */
static inline uint8_t* bat_ring_write_begin(bat_ring* r) {
    const uint64_t idx = r->hdr->wr_count;
    const uint32_t s = (uint32_t)(idx % r->hdr->nslots);
    bat_slot_hdr* sh = bat_ring_slot(r, s);
    __atomic_store_n(&sh->seq, sh->seq + 1u, __ATOMIC_RELAXED); /* now odd: busy */
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return bat_ring_payload(r, s);
}

static inline void bat_ring_write_end(bat_ring* r, uint32_t size, uint64_t t_capture) {
    const uint64_t idx = r->hdr->wr_count;
    const uint32_t s = (uint32_t)(idx % r->hdr->nslots);
    bat_slot_hdr* sh = bat_ring_slot(r, s);
    sh->size = size;
    sh->t_capture = t_capture;
    sh->t_publish = bat_ring_now_ns();
    sh->frame_idx = idx;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    __atomic_store_n(&sh->seq, sh->seq + 1u, __ATOMIC_RELAXED); /* even: stable */
    __atomic_store_n(&r->hdr->latest, s, __ATOMIC_RELEASE);
    __atomic_store_n(&r->hdr->wr_count, idx + 1u, __ATOMIC_RELEASE);
}

/* Reader: copy the newest complete frame into `buf`.
 * Returns 1 and fills `meta` (if non-NULL) on success, 0 if the ring holds no
 * frame yet, -1 if every attempt raced the writer or the frame is larger than
 * `bufsz`. The caller de-duplicates via meta->frame_idx. */
static inline int bat_ring_read_latest(const bat_ring* r, uint8_t* buf, size_t bufsz,
                                       bat_slot_hdr* meta) {
    for (int attempt = 0; attempt < 8; ++attempt) {
        const uint32_t idx = __atomic_load_n(&r->hdr->latest, __ATOMIC_ACQUIRE);
        if (idx == BAT_RING_NO_FRAME) return 0;
        if (idx >= r->hdr->nslots) return -1;

        bat_slot_hdr* sh = bat_ring_slot(r, idx);
        const uint32_t s0 = __atomic_load_n(&sh->seq, __ATOMIC_ACQUIRE);
        if (s0 & 1u) continue; /* writer mid-copy; latest may move, retry */

        bat_slot_hdr m;
        memcpy(&m, sh, sizeof(m));
        if (m.size > bufsz || m.size > r->hdr->slot_size) return -1;
        memcpy(buf, bat_ring_payload(r, idx), m.size);

        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        const uint32_t s1 = __atomic_load_n(&sh->seq, __ATOMIC_ACQUIRE);
        if (s0 == s1) {
            if (meta) *meta = m;
            return 1;
        }
    }
    return -1;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BAT_RING_H */
