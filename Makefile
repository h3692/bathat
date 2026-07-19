# Native build for QNX, compiled directly on the target (e.g. Raspberry Pi 5)
# with its own gcc/g++ -- no host SDP / cross-compilation.
#
# Dependencies (install once via apk on the Pi):
#   sudo apk add qnx-screen-dev qnx-sensor-framework-dev
# which provide the headers under /usr/include/{camera,screen}/ and the
# camapi + screen link libraries.
#
# Build:  make            -> ./bathat
#         make bat_audio  -> ./bat_audio (spatial-audio process)
# Test:   make test       -> runs the host unit test in tests/
# Clean:  make clean

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -O2 -Wall -Wextra -Isrc -Icommon
LDLIBS   ?= -lcamapi -lscreen

SRCS := src/main.cpp src/camera.cpp src/frameslot.cpp src/composite.cpp src/depthview.cpp src/detview.cpp src/rectview.cpp src/display.cpp
OBJS := $(SRCS:.cpp=.o)
HDRS := $(wildcard src/*.h common/*.h audio/*.h)

# bat_audio needs io-audio (QSA) only on the target; elsewhere the QSA sink is
# a stub and the WAV sink does the work.
AUDIO_SRCS := audio/main.cpp audio/synth.cpp audio/fusion.cpp \
	audio/speech.cpp audio/wavfilesink.cpp audio/qsasink.cpp
AUDIO_OBJS := $(AUDIO_SRCS:.cpp=.o)
ifeq ($(shell uname -s),QNX)
AUDIO_LDLIBS := -lasound
endif

bathat: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDLIBS)

bat_audio: $(AUDIO_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(AUDIO_OBJS) $(AUDIO_LDLIBS)

%.o: %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY: test clean
test:
	$(MAKE) -C tests

clean:
	rm -f bathat bat_audio $(OBJS) $(AUDIO_OBJS)
