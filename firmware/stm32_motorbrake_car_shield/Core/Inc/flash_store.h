/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : flash_store.h
 * @brief          : Persist the start/stop servo angles in on-chip flash.
 ******************************************************************************
 * @attention
 *
 * Stores the two servo angles (start = brake OFF/released, stop = brake
 * ON/engaged) in the last 2 KB page of flash bank 2 so they survive a reset.
 *
 * NOTE: the G474 has no read-while-write, so a page erase (~22 ms) stalls the
 * whole CPU — interrupts included — for its duration. Callers must therefore
 * debounce FlashStore_SaveAngles() (see FLASH_WRITE_QUIET_MS in main.c) so a
 * burst of /servo_command frames does not trigger back-to-back erase stalls
 * that would make CAN look unresponsive.
 *
 * This module is a self-contained black box: it talks only to the STM32 flash
 * and communicates through its arguments / return value. It touches none of the
 * application's shared globals.
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef INC_FLASH_STORE_H_
#define INC_FLASH_STORE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Recall the persisted start/stop angles.
  * @param  start_out     : receives the released (brake OFF) angle, 0..180°.
  * @param  stop_out      : receives the engaged  (brake ON)  angle, 0..180°.
  * @param  start_default : value written to *start_out when no valid stored
  *                         start angle exists.
  * @param  stop_default  : value written to *stop_out when no valid stored
  *                         stop angle exists.
  * @note   Falls back to the supplied defaults for either angle if the page was
  *         never written or the stored value is out of range. The defaults are
  *         owned by the application (see main.c) so this lib stays pure mechanism.
  */
void FlashStore_LoadAngles(float *start_out, float *stop_out,
                           float start_default, float stop_default);

/**
  * @brief  Erase the user page and store the start/stop angles.
  * @param  start_deg : released (brake OFF) angle to persist.
  * @param  stop_deg  : engaged  (brake ON)  angle to persist.
  * @retval 1 if both records read back exactly as written, 0 otherwise.
  * @note   The ~22 ms page erase stalls the whole CPU (no RWW on the G474);
  *         debounce calls to this from the caller.
  */
uint8_t FlashStore_SaveAngles(float start_deg, float stop_deg);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_STORE_H_ */
