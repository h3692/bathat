# Host unit tests (no QNX/hardware needed). Run on your dev/build machine:
#   make -C tests
# Covers the pure NV12 composite logic and the bat_ring shared-memory format,
# including a C-writer -> Python-reader interop check.
CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -O2 -Wall -Wextra
PYTHON   ?= python3

.PHONY: check clean
check: test_composite test_ring
	./test_composite
	./test_ring test_ring.ring
	$(PYTHON) test_ring_read.py test_ring.ring
	rm -f test_ring.ring

test_composite: test_composite.cpp ../src/composite.cpp ../src/composite.h
	$(CXX) $(CXXFLAGS) -I../src -o $@ test_composite.cpp ../src/composite.cpp

test_ring: test_ring.cpp ../common/bat_ring.h
	$(CXX) $(CXXFLAGS) -I../common -o $@ test_ring.cpp

clean:
	rm -f test_composite test_ring test_ring.ring
