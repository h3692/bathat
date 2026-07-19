# Native build for QNX, compiled directly on the target (e.g. Raspberry Pi 5)
# with its own gcc/g++ -- no host SDP / cross-compilation.
#
# Dependencies (install once via apk on the Pi):
#   sudo apk add qnx-screen-dev qnx-sensor-framework-dev
# which provide the headers under /usr/include/{camera,screen}/ and the
# camapi + screen link libraries.
#
# Build:  make          -> ./bathat
# Test:   make test     -> runs the host unit test in tests/
# Clean:  make clean

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -O2 -Wall -Wextra -Isrc
LDLIBS   ?= -lcamapi -lscreen

SRCS := src/main.cpp src/camera.cpp src/frameslot.cpp src/composite.cpp src/display.cpp
OBJS := $(SRCS:.cpp=.o)

bathat: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY: test clean
test:
	$(MAKE) -C tests

clean:
	rm -f bathat $(OBJS)
