#include "display.h"

#include <cstdio>

bool Display::init(int width, int height) {
    width_ = width;
    height_ = height;

    if (screen_create_context(&ctx_, SCREEN_APPLICATION_CONTEXT) != 0) {
        std::perror("screen_create_context");
        return false;
    }
    if (screen_create_window(&win_, ctx_) != 0) {
        std::perror("screen_create_window");
        return false;
    }

    int format = SCREEN_FORMAT_NV12;
    if (screen_set_window_property_iv(win_, SCREEN_PROPERTY_FORMAT, &format) != 0) {
        std::perror("SCREEN_PROPERTY_FORMAT");
        return false;
    }

    // CPU-writable, composited by Screen.
    int usage = SCREEN_USAGE_WRITE | SCREEN_USAGE_NATIVE;
    if (screen_set_window_property_iv(win_, SCREEN_PROPERTY_USAGE, &usage) != 0) {
        std::perror("SCREEN_PROPERTY_USAGE");
        return false;
    }

    int size[2] = {width, height};
    if (screen_set_window_property_iv(win_, SCREEN_PROPERTY_SIZE, size) != 0) {
        std::perror("SCREEN_PROPERTY_SIZE");
        return false;
    }
    if (screen_set_window_property_iv(win_, SCREEN_PROPERTY_BUFFER_SIZE, size) != 0) {
        std::perror("SCREEN_PROPERTY_BUFFER_SIZE");
        return false;
    }

    if (screen_create_window_buffers(win_, 1) != 0) {
        std::perror("screen_create_window_buffers");
        return false;
    }

    screen_buffer_t bufs[1];
    if (screen_get_window_property_pv(win_, SCREEN_PROPERTY_RENDER_BUFFERS,
                                      reinterpret_cast<void**>(bufs)) != 0) {
        std::perror("SCREEN_PROPERTY_RENDER_BUFFERS");
        return false;
    }
    buf_ = bufs[0];
    return true;
}

bool Display::begin_frame(Nv12Dest& dst) {
    if (!buf_) return false;

    void* ptr = nullptr;
    int stride = 0;
    if (screen_get_buffer_property_pv(buf_, SCREEN_PROPERTY_POINTER, &ptr) != 0) return false;
    if (screen_get_buffer_property_iv(buf_, SCREEN_PROPERTY_STRIDE, &stride) != 0) return false;
    if (!ptr) return false;

    uint8_t* base = static_cast<uint8_t*>(ptr);
    dst.y = base;
    // NV12: the UV plane follows the Y plane. VERIFY on hardware -- some Screen
    // configs pad between planes; if chroma looks shifted, query the actual
    // plane offset instead of assuming stride*height.
    dst.uv = base + static_cast<size_t>(stride) * height_;
    dst.width = width_;
    dst.height = height_;
    dst.y_stride = stride;
    dst.uv_stride = stride;
    return true;
}

void Display::end_frame() {
    if (!win_ || !buf_) return;
    int rect[4] = {0, 0, width_, height_};
    screen_post_window(win_, buf_, 1, rect, 0);
}

void Display::shutdown() {
    if (win_) {
        screen_destroy_window(win_);
        win_ = nullptr;
    }
    if (ctx_) {
        screen_destroy_context(ctx_);
        ctx_ = nullptr;
    }
    buf_ = nullptr;
}
