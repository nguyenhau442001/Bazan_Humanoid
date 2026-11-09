// Target entry: main.c calls app_start() once the peripherals are initialised.
#include "app.h"
#include "main.h"

#ifndef MICO_TEST_HOOKS
#define MICO_TEST_HOOKS 0
#endif

extern "C" void app_start(void) {
  mico::app::create_tasks(MICO_TEST_HOOKS != 0);
  vTaskStartScheduler();   // never returns: all task memory is static, so it cannot fail for lack of heap
  Error_Handler();
}
