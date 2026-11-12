/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : flash_store.c
 * @brief          : Persist the start/stop servo angles in on-chip flash.
 ******************************************************************************
 * @attention
 *
 * Two 64-bit records in the last 2 KB page of bank 2:
 *   DW0: [31:0] magic, [63:32] start_deg float
 *   DW1: [31:0] stop_deg float, [63:32] magic
 * Both records carry the magic so a stale page from a previous firmware layout
 * is rejected and the caller falls back to the defaults.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "flash_store.h"
#include "main.h"     /* HAL flash API + __IO */
#include <string.h>   /* memcpy() for the float<->uint32 bit shuffles */

/* Private define ------------------------------------------------------------*/
/* MAGIC bumped from the old single-angle format so a stale flash page from the
 * previous firmware is rejected and we fall back to the defaults. */
#define FLASH_USER_ADDR       0x0807F800UL
#define FLASH_USER_BANK       FLASH_BANK_2
#define FLASH_USER_PAGE       127u
#define FLASH_USER_MAGIC      0xB7A4E002UL

/* Private function prototypes -----------------------------------------------*/
static uint64_t flash_pack_dw0(float f);
static uint64_t flash_pack_dw1(float f);

/* Private functions ---------------------------------------------------------*/
/* Pack a float + the magic into one 64-bit record. The float occupies the upper
 * 32 bits and the magic the lower 32 (DW0); the stop record stores them swapped
 * (DW1) so both records carry a magic to validate against. */
static uint64_t flash_pack_dw0(float f) {   /* [31:0]=magic, [63:32]=float */
    uint32_t bits;
    memcpy(&bits, &f, sizeof(float));
    return ((uint64_t) bits << 32) | (uint64_t) FLASH_USER_MAGIC;
}
static uint64_t flash_pack_dw1(float f) {   /* [31:0]=float, [63:32]=magic */
    uint32_t bits;
    memcpy(&bits, &f, sizeof(float));
    return ((uint64_t) FLASH_USER_MAGIC << 32) | (uint64_t) bits;
}

/* Exported functions --------------------------------------------------------*/
void FlashStore_LoadAngles(float *start_out, float *stop_out,
                           float start_default, float stop_default) {
    *start_out = start_default;
    *stop_out  = stop_default;

    uint64_t dw0 = *(__IO uint64_t *) FLASH_USER_ADDR;
    uint64_t dw1 = *(__IO uint64_t *) (FLASH_USER_ADDR + 8u);
    if ((uint32_t) (dw0 & 0xFFFFFFFFUL) != FLASH_USER_MAGIC ||
        (uint32_t) (dw1 >> 32)          != FLASH_USER_MAGIC) {
        return;  /* never written / wrong format -> keep defaults */
    }

    uint32_t sbits = (uint32_t) (dw0 >> 32);
    uint32_t tbits = (uint32_t) (dw1 & 0xFFFFFFFFUL);
    float s, t;
    memcpy(&s, &sbits, sizeof(float));
    memcpy(&t, &tbits, sizeof(float));
    if (s >= 0.0f && s <= 180.0f) *start_out = s;
    if (t >= 0.0f && t <= 180.0f) *stop_out  = t;
}

uint8_t FlashStore_SaveAngles(float start_deg, float stop_deg) {
    uint64_t dw0 = flash_pack_dw0(start_deg);
    uint64_t dw1 = flash_pack_dw1(stop_deg);

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_err = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_USER_BANK;
    erase.Page      = FLASH_USER_PAGE;
    erase.NbPages   = 1;

    HAL_StatusTypeDef st = HAL_ERROR;
    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_ADDR, dw0);
        if (st == HAL_OK) {
            st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_ADDR + 8u, dw1);
        }
    }
    HAL_FLASH_Lock();

    /* Verify: both programmed doublewords must read back exactly. */
    return (st == HAL_OK &&
            *(__IO uint64_t *) FLASH_USER_ADDR        == dw0 &&
            *(__IO uint64_t *) (FLASH_USER_ADDR + 8u) == dw1) ? 1u : 0u;
}
