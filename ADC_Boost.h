/**
 ************************************************************************************************
 * @file       ADC_Boost.h
 * @author     Steve Ding, Colin Gallacher
 * @version    V1.0.0
 * @date       11-December-2018
 * @brief      ADC modification for Haply M0 and Arduino Zero SAMD boards. Extends ADC to 12-bit
 *             resolution
 ************************************************************************************************
 * @attention
 *
 *
 ************************************************************************************************
 */

#include "Arduino.h"

void ADC_Boost(){
  ADC->CTRLA.bit.ENABLE = 0;                     // Disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );        // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |   // Divide Clock by 64.
                   ADC_CTRLB_RESSEL_12BIT;       // Result on 12 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                      // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1;                     // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );        // Wait for synchronization
}

