# M0 integration test: the firmware's FreeRTOS tasks on the FreeRTOS POSIX port.
#   make -C firmware/stm32f411/test/posix test
FW = ../..
CORE = ../../../../lib/core
RTOS = $(FW)/FreeRTOS/Source
CXX ?= g++
CC ?= gcc
SAN ?= -fsanitize=undefined
INC = -I. -I$(FW)/app -I$(RTOS)/include -I$(RTOS)/portable/ThirdParty/GCC/Posix \
      -I$(RTOS)/portable/ThirdParty/GCC/Posix/utils \
      $(addprefix -I$(CORE)/,controller trajectory joint_manager link safety timing generated .)
RTOS_SRC = $(RTOS)/tasks.c $(RTOS)/list.c $(RTOS)/queue.c $(RTOS)/portable/MemMang/heap_4.c \
           $(RTOS)/portable/ThirdParty/GCC/Posix/port.c $(RTOS)/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c
CORE_SRC = $(wildcard $(CORE)/controller/*.cpp $(CORE)/trajectory/*.cpp $(CORE)/joint_manager/*.cpp \
           $(CORE)/link/*.cpp $(CORE)/safety/*.cpp $(CORE)/timing/*.cpp)
APP_SRC = $(FW)/app/tasks.cpp $(FW)/app/diagnostics.cpp board_sim.cpp pty_bridge.cpp test_m0.cpp

test: build/test_m0
	./build/test_m0
	./build/test_m0 --watchdog-reset

RTOS_OBJ = $(addprefix build/rtos/,$(notdir $(RTOS_SRC:.c=.o)))
vpath %.c $(sort $(dir $(RTOS_SRC)))

build/rtos/%.o: %.c FreeRTOSConfig.h
	mkdir -p build/rtos
	$(CC) -O1 -g $(INC) -c $< -o $@

build/rtos.a: $(RTOS_OBJ)
	ar rcs $@ $^

build/test_m0: build/rtos.a $(CORE_SRC) $(APP_SRC) $(wildcard *.h $(FW)/app/*.h $(CORE)/*/*.h)
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra -Werror -Wno-missing-field-initializers $(SAN) $(INC) \
	  $(CORE_SRC) $(APP_SRC) build/rtos.a -o $@ -lpthread -lm

# The real host tool's m0-accept against the simulated MCU behind a pty (checks the tool, not the timing).
accept-sim: build/test_m0
	./build/test_m0 --pty > build/pty.txt 2> build/sim.err & echo $$! > build/sim.pid; sleep 1; \
	python3 ../../tools/mico_link.py --port $$(head -1 build/pty.txt) m0-accept --minutes 0.2 \
	  --report build/m0_accept_sim.json; rc=$$?; kill $$(cat build/sim.pid) 2>/dev/null || true; exit $$rc   # same pid after the simulated reset

clean:
	rm -rf build

.PHONY: test accept-sim clean
