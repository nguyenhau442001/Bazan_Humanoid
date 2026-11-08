# Host build of the hardware-independent control core and its tests. No MCU toolchain needed.
#   make -C lib/core test
CXX ?= g++
SAN ?= -fsanitize=address,undefined
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wextra -Werror -Wno-missing-field-initializers $(SAN)
DIRS = controller trajectory joint_manager link safety timing generated .
INC = $(addprefix -I,$(DIRS))
SRC = $(wildcard controller/*.cpp trajectory/*.cpp joint_manager/*.cpp link/*.cpp safety/*.cpp timing/*.cpp)
HDR = $(wildcard */*.h *.h)

test: build/test_core build/test_safety build/test_threads
	./build/test_core
	./build/test_safety
	./build/test_threads

build/test_core: $(SRC) $(HDR) test/test_core.cpp
	mkdir -p build
	$(CXX) $(CXXFLAGS) $(INC) $(SRC) test/test_core.cpp -o $@ -lm

build/test_safety: $(SRC) $(HDR) test/test_safety.cpp
	mkdir -p build
	$(CXX) $(CXXFLAGS) $(INC) $(SRC) test/test_safety.cpp -o $@ -lm

# Two real threads on the lock-free ring and the seqlock, under ThreadSanitizer.
build/test_threads: $(HDR) test/test_threads.cpp
	mkdir -p build
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=thread $(INC) test/test_threads.cpp -o $@ -lpthread

clean:
	rm -rf build

.PHONY: test clean
