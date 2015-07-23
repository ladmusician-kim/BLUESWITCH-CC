/**************************************************************************************************
  Filename:       hal_bs_key.h
  Revised:        $Date: 2014-02-17
  Revision:       $Revision: $

  Description:    This file contains the interface to the Blue Switch KEY Service.

**************************************************************************************************/

#ifndef HAL_KEY_H
#define HAL_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* Interrupt option - Enable or disable */
#define HAL_BS_KEY_INTERRUPT_DISABLE    0x00
#define HAL_BS_KEY_INTERRUPT_ENABLE     0x01

/* Key state - shift or nornal */
#define HAL_BS_KEY_STATE_NORMAL          0x00
#define HAL_BS_KEY_STATE_SHIFT           0x01

#define HAL_BS_KEY_BTN1                 0x01
#define HAL_BS_KEY_BTN2                 0x02
#define HAL_BS_KEY_BTN3                 0x04
#define HAL_BS_KEY_BTN4                 0x08
  
/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/
typedef void (*halBsKeyCBack_t) (uint8 keys, uint8 state);

/**************************************************************************************************
 *                                             GLOBAL VARIABLES
 **************************************************************************************************/
extern bool Hal_BsKeyIntEnable;

/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize the Key Service
 */
extern void HalBsKeyInit( void );

/*
 * Configure the Key Service
 */
extern void HalBsKeyConfig( bool interruptEnable, const halBsKeyCBack_t cback);

/*
 * Read the Key status
 */
extern uint8 HalBsKeyRead( void);

/*
 * Enter sleep mode, store important values
 */
extern void HalBsKeyEnterSleep ( void );

/*
 * Exit sleep mode, retore values
 */
extern uint8 HalBsKeyExitSleep ( void );

/*
 * This is for internal used by hal_driver
 */
extern void HalBsKeyPoll ( void );

/*
 * This is for internal used by hal_sleep
 */
extern bool HalBsKeyPressed( void );

extern uint8 hal_bs_key_keys(void);                                           

extern uint8 hal_bs_key_int_keys(void);

/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
