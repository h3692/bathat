# Host unit test for the pure NV12 composite logic (no QNX/hardware needed).
# Run on your dev/build machine:   make -C tests
CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -O2 -Wall -Wextra
SRC       = test_composite.cpp ../src/composite.cpp

.PHONY: check clean
check: test_composite
	./test_composite

test_composite: $(SRC) ../src/composite.h
	$(CXX) $(CXXFLAGS) -I../src -o $@ $(SRC)

clean:
	rm -f test_composite
