/* FreeRTOS configuration for Mico on the STM32F411 Black Pill (96 MHz Cortex-M4F). */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
extern uint32_t SystemCoreClock;
uint32_t mico_runtime_counter(void);   /* TIM5, 1 MHz, 32-bit: also the control loop's clock */
void mico_trace_malloc(void);          /* counts allocations after the scheduler starts */

#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configCPU_CLOCK_HZ                       (SystemCoreClock)
#define configTICK_RATE_HZ                       1000
#define configMAX_PRIORITIES                     7
#define configMINIMAL_STACK_SIZE                 128
#define configMAX_TASK_NAME_LEN                  12
#define configIDLE_TASK_NAME                     "IDLE"
#define configTICK_TYPE_WIDTH_IN_BITS            TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            1
#define configQUEUE_REGISTRY_SIZE                0
#define configUSE_TIME_SLICING                   1
#define configENABLE_BACKWARD_COMPATIBILITY      0

/* Memory: tasks are static. The heap exists for micro-ROS (M1); in M0 nothing may allocate after start. */
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configTOTAL_HEAP_SIZE                    (24 * 1024)
#define configAPPLICATION_ALLOCATED_HEAP         0
#define traceMALLOC(pvAddress, uiSize)           mico_trace_malloc()

/* Hooks and checks */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_MALLOC_FAILED_HOOK             1
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0

/* Run-time statistics for the per-task CPU load in Diag */
#define configUSE_TRACE_FACILITY                 1
#define configGENERATE_RUN_TIME_STATS            1
#define configRUN_TIME_COUNTER_TYPE              uint32_t
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() /* TIM5 is started in main() before the scheduler */
#define portGET_RUN_TIME_COUNTER_VALUE()         mico_runtime_counter()

#define configUSE_TIMERS                         0
#define configUSE_CO_ROUTINES                    0

/* Interrupt priorities: 4 bits on the STM32F4. ISRs at 5..15 may call FromISR APIs. */
#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY          (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define configASSERT(x)                          do { if ((x) == 0) { __asm volatile("cpsid i"); for (;;) {} } } while (0)

#define INCLUDE_vTaskPrioritySet                 0
#define INCLUDE_uxTaskPriorityGet                0
#define INCLUDE_vTaskDelete                      0
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetCurrentTaskHandle        1

/* Map the port's handlers onto the CMSIS vector names (SysTick belongs to FreeRTOS; HAL ticks on TIM11). */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
