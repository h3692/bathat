#include "wavfilesink.h"

#include <cstring>
#include <utility>

#include "synth.h"

namespace {

void put_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

}  // namespace

WavFileSink::WavFileSink(std::string path) : path_(std::move(path)) {}

WavFileSink::~WavFileSink() { stop(); }

bool WavFileSink::start() {
    file_ = std::fopen(path_.c_str(), "wb");
    if (!file_) return false;

    uint8_t hdr[44];
    std::memcpy(hdr, "RIFF", 4);
    put_u32(hdr + 4, 36);  // patched on stop()
    std::memcpy(hdr + 8, "WAVE", 4);
    std::memcpy(hdr + 12, "fmt ", 4);
    put_u32(hdr + 16, 16);
    put_u16(hdr + 20, 1);  // PCM
    put_u16(hdr + 22, 2);  // stereo
    put_u32(hdr + 24, synth::kSampleRate);
    put_u32(hdr + 28, synth::kSampleRate * 4);  // byte rate
    put_u16(hdr + 32, 4);                       // block align
    put_u16(hdr + 34, 16);                      // bits per sample
    std::memcpy(hdr + 36, "data", 4);
    put_u32(hdr + 40, 0);  // patched on stop()
    return std::fwrite(hdr, 1, sizeof(hdr), file_) == sizeof(hdr);
}

bool WavFileSink::write(const int16_t* frames, int nframes) {
    if (!file_) return false;
    const size_t bytes = static_cast<size_t>(nframes) * 2 * sizeof(int16_t);
    if (std::fwrite(frames, 1, bytes, file_) != bytes) return false;
    data_bytes_ += static_cast<uint32_t>(bytes);
    return true;
}

void WavFileSink::stop() {
    if (!file_) return;
    uint8_t size4[4];
    put_u32(size4, 36 + data_bytes_);
    std::fseek(file_, 4, SEEK_SET);
    std::fwrite(size4, 1, 4, file_);
    put_u32(size4, data_bytes_);
    std::fseek(file_, 40, SEEK_SET);
    std::fwrite(size4, 1, 4, file_);
    std::fclose(file_);
    file_ = nullptr;
}
