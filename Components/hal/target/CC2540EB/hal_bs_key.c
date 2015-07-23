/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2013-09-20 11:53:10 -0700 (Fri, 20 Sep 2013) $
  Revision:       $Revision: 35401 $

  Description:    This file contains the interface to the HAL KEY Service.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that t he Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/
/*********************************************************************
 NOTE: If polling is used, the hal_driver task schedules the KeyRead()
       to occur every 100ms.  This should be long enough to naturally
       debounce the keys.  The KeyRead() function remembers the key
       state of the previous poll and will only return a non-zero
       value if the key state changes.

 NOTE: If interrupts are used, the KeyRead() function is scheduled
       25ms after the interrupt occurs by the ISR.  This delay is used
       for key debouncing.  The ISR disables any further Key interrupt
       until KeyRead() is executed.  KeyRead() will re-enable Key
       interrupts after executing.  Unlike polling, when interrupts
       are enabled, the previous key state is not remembered.  This
       means that KeyRead() will return the current state of the keys
       (not a change in state of the keys).

 NOTE: If interrupts are used, the KeyRead() fucntion is scheduled by
       the ISR.  Therefore, the joystick movements will only be detected
       during a pushbutton interrupt caused by S1 or the center joystick
       pushbutton.

 NOTE: When a switch like S1 is pushed, the S1 signal goes from a normally
       high state to a low state.  This transition is typically clean.  The
       duration of the low state is around 200ms.  When the signal returns
       to the high state, there is a high likelihood of signal bounce, which
       causes a unwanted interrupts.  Normally, we would set the interrupt
       edge to falling edge to generate an interrupt when S1 is pushed, but
       because of the signal bounce, it is better to set the edge to rising
       edge to generate an interrupt when S1 is released.  The debounce logic
       can then filter out the signal bounce.  The result is that we typically
       get only 1 interrupt per button push.  This mechanism is not totally
       foolproof because occasionally, signal bound occurs during the falling
       edge as well.  A similar mechanism is used to handle the joystick
       pushbutton on the DB.  For the EB, we do not have independent control
       of the interrupt edge for the S1 and center joystick pushbutton.  As
       a result, only one or the other pushbuttons work reasonably well with
       interrupts.  The default is the make the S1 switch on the EB work more
       reliably.

*********************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_bs_key.h"
#include "osal.h"

#if (defined HAL_BS_KEY) && (HAL_BS_KEY == TRUE)

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_BS_KEY_RISING_EDGE   0
#define HAL_BS_KEY_FALLING_EDGE  1

#define HAL_BS_KEY_DEBOUNCE_VALUE  25

/* CPU port interrupt */
#define HAL_BS_KEY_CPU_PORT_0_IF P0IF
#define HAL_BS_KEY_CPU_PORT_2_IF P2IF

/* BTN1 key is at P0_1 */
#define HAL_BS_KEY_BTN1_PORT   P0
#define HAL_BS_KEY_BTN1_BIT    BV(5)
#define HAL_BS_KEY_BTN1_SEL    P0SEL
#define HAL_BS_KEY_BTN1_DIR    P0DIR

/* edge interrupt */
#define HAL_BS_KEY_BTN1_EDGEBIT  BV(0)
#define HAL_BS_KEY_BTN1_EDGE     HAL_BS_KEY_FALLING_EDGE

/* Button interrupts */
#define HAL_BS_KEY_BTN1_IEN       IEN1  /* CPU interrupt mask register */
#define HAL_BS_KEY_BTN1_ICTL      P0IEN /* Port Interrupt Control register */
#define HAL_BS_KEY_BTN1_ICTLBIT   BV(5) /* P0IEN - P0.1 enable/disable bit */
#define HAL_BS_KEY_BTN1_IENBIT    BV(5) /* Mask bit for all of Port_0 */
#define HAL_BS_KEY_BTN1_PXIFG     P0IFG



/* BTN2 key is at P0_4 */
#define HAL_BS_KEY_BTN2_PORT   P0
#define HAL_BS_KEY_BTN2_BIT    BV(6)
#define HAL_BS_KEY_BTN2_SEL    P0SEL
#define HAL_BS_KEY_BTN2_DIR    P0DIR

/* edge interrupt */
#define HAL_BS_KEY_BTN2_EDGEBIT  BV(0)
#define HAL_BS_KEY_BTN2_EDGE     HAL_BS_KEY_FALLING_EDGE

/* Button interrupts */
#define HAL_BS_KEY_BTN2_IEN       IEN1  /* CPU interrupt mask register */
#define HAL_BS_KEY_BTN2_ICTL      P0IEN /* Port Interrupt Control register */
#define HAL_BS_KEY_BTN2_ICTLBIT   BV(6) /* P0IEN - P0.4 enable/disable bit */
#define HAL_BS_KEY_BTN2_IENBIT    BV(5) /* Mask bit for all of Port_0 */
#define HAL_BS_KEY_BTN2_PXIFG     P0IFG



/* BTN3 key is at P0_5 */
#define HAL_BS_KEY_BTN3_PORT   P0
#define HAL_BS_KEY_BTN3_BIT    BV(4)
#define HAL_BS_KEY_BTN3_SEL    P0SEL
#define HAL_BS_KEY_BTN3_DIR    P0DIR

/* edge interrupt */
#define HAL_BS_KEY_BTN3_EDGEBIT  BV(0)
#define HAL_BS_KEY_BTN3_EDGE     HAL_BS_KEY_FALLING_EDGE

/* Button interrupts */
#define HAL_BS_KEY_BTN3_IEN       IEN1  /* CPU interrupt mask register */
#define HAL_BS_KEY_BTN3_ICTL      P0IEN /* Port Interrupt Control register */
#define HAL_BS_KEY_BTN3_ICTLBIT   BV(4) /* P0IEN - P0.5 enable/disable bit */
#define HAL_BS_KEY_BTN3_IENBIT    BV(5) /* Mask bit for all of Port_0 */
#define HAL_BS_KEY_BTN3_PXIFG     P0IFG



/* BTN4 key is at P0_6 */
#define HAL_BS_KEY_BTN4_PORT   P0
#define HAL_BS_KEY_BTN4_BIT    BV(1)
#define HAL_BS_KEY_BTN4_SEL    P0SEL
#define HAL_BS_KEY_BTN4_DIR    P0DIR

/* edge interrupt */
#define HAL_BS_KEY_BTN4_EDGEBIT  BV(0)
#define HAL_BS_KEY_BTN4_EDGE     HAL_BS_KEY_FALLING_EDGE

/* Button interrupts */
#define HAL_BS_KEY_BTN4_IEN       IEN1  /* CPU interrupt mask register */
#define HAL_BS_KEY_BTN4_ICTL      P0IEN /* Port Interrupt Control register */
#define HAL_BS_KEY_BTN4_ICTLBIT   BV(1) /* P0IEN - P0.6 enable/disable bit */
#define HAL_BS_KEY_BTN4_IENBIT    BV(5) /* Mask bit for all of Port_0 */
#define HAL_BS_KEY_BTN4_PXIFG     P0IFG


#endif

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halBsKeySavedKeys;     /* used to store previous key state in polling mode */
static halBsKeyCBack_t pHalBsKeyProcessFunction;
static uint8 HalBsKeyConfigured;
bool Hal_BsKeyIntEnable;            /* interrupt enable/disable flag */

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessBsKeyInterrupt(void);

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalBsKeyInit( void )
{
  halBsKeySavedKeys = 0;  // Initialize previous key to 0.

  // BTN1
  HAL_BS_KEY_BTN1_SEL &= ~(HAL_BS_KEY_BTN1_BIT);    /* Set pin function to GPIO */
  HAL_BS_KEY_BTN1_DIR &= ~(HAL_BS_KEY_BTN1_BIT);    /* Set pin direction to Input */
  // BTN2
  HAL_BS_KEY_BTN2_SEL &= ~(HAL_BS_KEY_BTN2_BIT);    /* Set pin function to GPIO */
  HAL_BS_KEY_BTN2_DIR &= ~(HAL_BS_KEY_BTN2_BIT);    /* Set pin direction to Input */
  // BTN3
  HAL_BS_KEY_BTN3_SEL &= ~(HAL_BS_KEY_BTN3_BIT);    /* Set pin function to GPIO */
  HAL_BS_KEY_BTN3_DIR &= ~(HAL_BS_KEY_BTN3_BIT);    /* Set pin direction to Input */
  // BTN4
  HAL_BS_KEY_BTN4_SEL &= ~(HAL_BS_KEY_BTN4_BIT);    /* Set pin function to GPIO */
  HAL_BS_KEY_BTN4_DIR &= ~(HAL_BS_KEY_BTN4_BIT);    /* Set pin direction to Input */

  /* Initialize callback function */
  pHalBsKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalBsKeyConfigured = FALSE;
}


/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
static uint16 intEnable;
void HalBsKeyConfig (bool interruptEnable, halBsKeyCBack_t cback)
{
  /* Enable/Disable Interrupt or */
  Hal_BsKeyIntEnable = interruptEnable;
  intEnable = interruptEnable;

  /* Register the callback fucntion */
  pHalBsKeyProcessFunction = cback;

  /* Determine if interrupt is enable or not */
  if (Hal_BsKeyIntEnable)
  {
    PICTL |= ( HAL_BS_KEY_BTN1_EDGEBIT | HAL_BS_KEY_BTN2_EDGEBIT | HAL_BS_KEY_BTN3_EDGEBIT | HAL_BS_KEY_BTN4_EDGEBIT );
    HAL_BS_KEY_BTN1_IEN |= ( HAL_BS_KEY_BTN1_IENBIT | HAL_BS_KEY_BTN2_IENBIT | HAL_BS_KEY_BTN3_IENBIT | HAL_BS_KEY_BTN4_IENBIT );
    
    // Rising/Falling edge configuratinn 
    PICTL &= ~(HAL_BS_KEY_BTN1_EDGEBIT);    // Clear the edge bit 
    // For falling edge, the bit must be set. 
  #if (HAL_BS_KEY_BTN1_EDGE == HAL_BS_KEY_FALLING_EDGE)
    PICTL |= HAL_BS_KEY_BTN1_EDGEBIT;
  #endif

    // Interrupt configuration:
    // - Enable interrupt generation at the port
    // - Enable CPU interrupt
    // - Clear any pending interrupt
    
    HAL_BS_KEY_BTN1_ICTL |= HAL_BS_KEY_BTN1_ICTLBIT;
    HAL_BS_KEY_BTN1_IEN |= HAL_BS_KEY_BTN1_IENBIT;
    HAL_BS_KEY_BTN1_PXIFG = ~(HAL_BS_KEY_BTN1_BIT);
    
    
    
    // Rising/Falling edge configuratinn 
    PICTL &= ~(HAL_BS_KEY_BTN2_EDGEBIT);    // Clear the edge bit 
    // For falling edge, the bit must be set. 
  #if (HAL_BS_KEY_BTN2_EDGE == HAL_BS_KEY_FALLING_EDGE)
    PICTL |= HAL_BS_KEY_BTN2_EDGEBIT;
  #endif

    // Interrupt configuration:
    // - Enable interrupt generation at the port
    // - Enable CPU interrupt
    // - Clear any pending interrupt
     
    HAL_BS_KEY_BTN2_ICTL |= HAL_BS_KEY_BTN2_ICTLBIT;
    HAL_BS_KEY_BTN2_IEN |= HAL_BS_KEY_BTN2_IENBIT;
    HAL_BS_KEY_BTN2_PXIFG = ~(HAL_BS_KEY_BTN2_BIT);
    
    
    
    // Rising/Falling edge configuratinn 
    PICTL &= ~(HAL_BS_KEY_BTN3_EDGEBIT);    // Clear the edge bit 
    // For falling edge, the bit must be set. 
    #if (HAL_BS_KEY_BTN3_EDGE == HAL_BS_KEY_FALLING_EDGE)
    PICTL |= HAL_BS_KEY_BTN3_EDGEBIT;
    #endif

    // Interrupt configuration:
    //  - Enable interrupt generation at the port
    //  - Enable CPU interrupt
    //  - Clear any pending interrupt
     
    HAL_BS_KEY_BTN3_ICTL |= HAL_BS_KEY_BTN3_ICTLBIT;
    HAL_BS_KEY_BTN3_IEN |= HAL_BS_KEY_BTN3_IENBIT;
    HAL_BS_KEY_BTN3_PXIFG = ~(HAL_BS_KEY_BTN3_BIT);
    
    
    
    // Rising/Falling edge configuratinn 
    PICTL &= ~(HAL_BS_KEY_BTN4_EDGEBIT);    // Clear the edge bit 
    // For falling edge, the bit must be set. 
    #if (HAL_BS_KEY_BTN4_EDGE == HAL_BS_KEY_FALLING_EDGE)
    PICTL |= HAL_BS_KEY_BTN4_EDGEBIT;
    #endif

    // Interrupt configuration:
    // - Enable interrupt generation at the port
    // - Enable CPU interrupt
    // - Clear any pending interrupt
     
    HAL_BS_KEY_BTN4_ICTL |= HAL_BS_KEY_BTN4_ICTLBIT;
    HAL_BS_KEY_BTN4_IEN |= HAL_BS_KEY_BTN4_IENBIT;
    HAL_BS_KEY_BTN4_PXIFG = ~(HAL_BS_KEY_BTN4_BIT);
    
    
    /*
    HAL_BS_KEY_BTN1_ICTL |= HAL_BS_KEY_BTN1_ICTLBIT; // enable interrupt generation at port 
    HAL_BS_KEY_BTN1_PXIFG = ~(HAL_BS_KEY_BTN1_ICTLBIT);  // Clear any pending interrupt 
    HAL_BS_KEY_BTN2_ICTL |= HAL_BS_KEY_BTN2_ICTLBIT; // enable interrupt generation at port     
    HAL_BS_KEY_BTN2_PXIFG = ~(HAL_BS_KEY_BTN2_ICTLBIT);  // Clear any pending interrupt 
    HAL_BS_KEY_BTN3_ICTL |= HAL_BS_KEY_BTN3_ICTLBIT; // enable interrupt generation at port     
    HAL_BS_KEY_BTN3_PXIFG = ~(HAL_BS_KEY_BTN3_ICTLBIT);  // Clear any pending interrupt 
    HAL_BS_KEY_BTN4_ICTL |= HAL_BS_KEY_BTN4_ICTLBIT; // enable interrupt generation at port     
    HAL_BS_KEY_BTN4_PXIFG = ~(HAL_BS_KEY_BTN4_ICTLBIT);  // Clear any pending interrupt 
    */
    
    
    // Do this only after the hal_key is configured - to work with sleep stuff 
    if (HalBsKeyConfigured == TRUE)
    {
      osal_stop_timerEx(Hal_TaskID, HAL_BS_KEY_EVENT);  // Cancel polling if active 
    }
  }
  else    /* Interrupts NOT enabled */
  {
    HAL_BS_KEY_BTN1_ICTL &= ~(HAL_BS_KEY_BTN1_ICTLBIT); /* don't generate interrupt */
    HAL_BS_KEY_BTN1_IEN &= ~(HAL_BS_KEY_BTN1_IENBIT);   /* Clear interrupt enable bit */
    
    HAL_BS_KEY_BTN2_ICTL &= ~(HAL_BS_KEY_BTN2_ICTLBIT); /* don't generate interrupt */
    HAL_BS_KEY_BTN2_IEN &= ~(HAL_BS_KEY_BTN2_IENBIT);   /* Clear interrupt enable bit */
    
    HAL_BS_KEY_BTN3_ICTL &= ~(HAL_BS_KEY_BTN3_ICTLBIT); /* don't generate interrupt */
    HAL_BS_KEY_BTN3_IEN &= ~(HAL_BS_KEY_BTN3_IENBIT);   /* Clear interrupt enable bit */
   
    HAL_BS_KEY_BTN4_ICTL &= ~(HAL_BS_KEY_BTN4_ICTLBIT); /* don't generate interrupt */
    HAL_BS_KEY_BTN4_IEN &= ~(HAL_BS_KEY_BTN4_IENBIT);   /* Clear interrupt enable bit */

    osal_set_event(Hal_TaskID, HAL_BS_KEY_EVENT);
  }

  /* Key now is configured */
  HalBsKeyConfigured = TRUE;
}


/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalBsKeyRead ( void )
{
  uint8 keys = 0;

  if ( !(HAL_BS_KEY_BTN1_PORT & HAL_BS_KEY_BTN1_BIT))    /* Key is active low */
  {
    keys |= HAL_BS_KEY_BTN1;
  }
  
  if ( !(HAL_BS_KEY_BTN2_PORT & HAL_BS_KEY_BTN2_BIT))    /* Key is active low */
  {
    keys |= HAL_BS_KEY_BTN2;
  }
  
  if ( !(HAL_BS_KEY_BTN3_PORT & HAL_BS_KEY_BTN3_BIT))    /* Key is active low */
  {
    keys |= HAL_BS_KEY_BTN3;
  }
  
  if ( !(HAL_BS_KEY_BTN4_PORT & HAL_BS_KEY_BTN4_BIT))    /* Key is active low */
  {
    keys |= HAL_BS_KEY_BTN4;
  }

  
  return keys;
}


/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalBsKeyPoll (void)
{
  uint8 keys = 0;
  uint8 notify = 0;

  if (!(HAL_BS_KEY_BTN1_PORT & HAL_BS_KEY_BTN1_BIT))  /* Key is active low */
  {
    keys |= HAL_BS_KEY_BTN1;
  }
  
  if (!(HAL_BS_KEY_BTN2_PORT & HAL_BS_KEY_BTN2_BIT))  /* Key is active low */
  {
    keys |= HAL_BS_KEY_BTN2;
  }
  
  if (!(HAL_BS_KEY_BTN3_PORT & HAL_BS_KEY_BTN3_BIT))  /* Key is active low */
  {
    keys |= HAL_BS_KEY_BTN3;
  }
  
  if (!(HAL_BS_KEY_BTN4_PORT & HAL_BS_KEY_BTN4_BIT))  /* Key is active low */
  {
    keys |= HAL_BS_KEY_BTN4;
  }
    

  /* If interrupts are not enabled, previous key status and current key status
   * are compared to find out if a key has changed status.
   */
  if (!Hal_BsKeyIntEnable)
  {
    if (keys == halBsKeySavedKeys)
    {
      /* Exit - since no keys have changed */
      return;
    }
    else
    {
      notify = 1;
    }
  }
  else
  {
    /* Key interrupt handled here */
    if (keys)
    {
      notify = 1;
    }
  }

  /* Store the current keys for comparation next time */
  halBsKeySavedKeys = keys;

  /* Invoke Callback if new keys were depressed */
  if (notify && (pHalBsKeyProcessFunction))
  {
    (pHalBsKeyProcessFunction) (keys, HAL_BS_KEY_STATE_NORMAL);

  }
}

/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessBsKeyInterrupt (void)
{
  bool valid=FALSE;

  if (HAL_BS_KEY_BTN1_PXIFG & HAL_BS_KEY_BTN1_BIT)  /* Interrupt Flag has been set */
  {
    HAL_BS_KEY_BTN1_PXIFG = ~(HAL_BS_KEY_BTN1_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  
  if (HAL_BS_KEY_BTN2_PXIFG & HAL_BS_KEY_BTN2_BIT)  /* Interrupt Flag has been set */
  {
    HAL_BS_KEY_BTN2_PXIFG = ~(HAL_BS_KEY_BTN2_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  
  if (HAL_BS_KEY_BTN3_PXIFG & HAL_BS_KEY_BTN3_BIT)  /* Interrupt Flag has been set */
  {
    HAL_BS_KEY_BTN3_PXIFG = ~(HAL_BS_KEY_BTN3_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  
  if (HAL_BS_KEY_BTN4_PXIFG & HAL_BS_KEY_BTN4_BIT)  /* Interrupt Flag has been set */
  {
    HAL_BS_KEY_BTN4_PXIFG = ~(HAL_BS_KEY_BTN4_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  
  if (valid)
  {
    osal_start_timerEx (Hal_TaskID, HAL_BS_KEY_EVENT, HAL_BS_KEY_DEBOUNCE_VALUE);
  }
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalBsKeyEnterSleep ( void )
{
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalBsKeyExitSleep ( void )
{
  /* Wake up and read keys */
  return ( HalBsKeyRead () );
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halBsKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halBsKeyPort0Isr, P0INT_VECTOR )
{
  HAL_ENTER_ISR();

  if ((HAL_BS_KEY_BTN1_PXIFG & HAL_BS_KEY_BTN1_BIT) || (HAL_BS_KEY_BTN2_PXIFG & HAL_BS_KEY_BTN2_BIT)
      || (HAL_BS_KEY_BTN3_PXIFG & HAL_BS_KEY_BTN3_BIT) || (HAL_BS_KEY_BTN4_PXIFG & HAL_BS_KEY_BTN4_BIT))
  {
    halProcessBsKeyInterrupt();
  }

  /*
    Clear the CPU interrupt flag for Port_0
    PxIFG has to be cleared before PxIF
  */
  
  HAL_BS_KEY_BTN1_PXIFG = 0;
  HAL_BS_KEY_BTN2_PXIFG = 0;
  HAL_BS_KEY_BTN3_PXIFG = 0;
  HAL_BS_KEY_BTN4_PXIFG = 0;

  HAL_BS_KEY_CPU_PORT_0_IF = 0;

  CLEAR_SLEEP_MODE();

  HAL_EXIT_ISR();

  return;
}
/**************************************************************************************************
**************************************************************************************************/
