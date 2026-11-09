/* Host (POSIX port) configuration for the integration test. Task-relevant settings match
 * Core/Inc/FreeRTOSConfig.h: tick, priorities, static allocation, run-time stats, the malloc trace.
 * Interrupt priorities, the Cortex-M handler names and the stack check do not apply on the host. */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
void mico_trace_malloc(void);

#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configTICK_RATE_HZ                       1000
#define configMAX_PRIORITIES                     7
#define configMINIMAL_STACK_SIZE                 (16384 / sizeof(StackType_t))   /* the port raises any smaller stack to PTHREAD_STACK_MIN */
#define configMAX_TASK_NAME_LEN                  12
#define configIDLE_TASK_NAME                     "IDLE"
#define configTICK_TYPE_WIDTH_IN_BITS            TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_MUTEXES                        1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_TIME_SLICING                   1
#define configENABLE_BACKWARD_COMPATIBILITY      0

#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configTOTAL_HEAP_SIZE                    (24 * 1024)
#define traceMALLOC(pvAddress, uiSize)           mico_trace_malloc()

#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCHECK_FOR_STACK_OVERFLOW           0
#define configUSE_MALLOC_FAILED_HOOK             1

#define configUSE_TRACE_FACILITY                 1
#define configGENERATE_RUN_TIME_STATS            1
#define configRUN_TIME_COUNTER_TYPE              uint32_t

#define configUSE_TIMERS                         0
#define configUSE_CO_ROUTINES                    0

#define configASSERT(x)                          do { if ((x) == 0) { vAssertCalled(__FILE__, __LINE__); } } while (0)
void vAssertCalled(const char* file, int line);

#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetCurrentTaskHandle        1

#include <limits.h>
#include <pthread.h>

#endif /* FREERTOS_CONFIG_H */
