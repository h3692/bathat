#ifndef BATHAT_DISPLAY_H
#define BATHAT_DISPLAY_H

#include <screen/screen.h>

#include "composite.h"

// A single QNX Screen window that we CPU-render the composited NV12 frame into.
// The compositor draws it to the display, which your VNC session mirrors.
class Display {
public:
    // Create an NV12 render buffer of width x height. The window itself is
    // sized to aspect-fit the attached display (Screen GPU-scales the buffer),
    // so composites wider or taller than the panel still show in full.
    bool init(int width, int height);
    void shutdown();

    // Map the current back buffer as an NV12 destination for compositing.
    bool begin_frame(Nv12Dest& dst);

    // Post the composited back buffer to the screen.
    void end_frame();

private:
    screen_context_t ctx_ = nullptr;
    screen_window_t win_ = nullptr;
    screen_buffer_t buf_ = nullptr;
    int width_ = 0, height_ = 0;
};

#endif  // BATHAT_DISPLAY_H
