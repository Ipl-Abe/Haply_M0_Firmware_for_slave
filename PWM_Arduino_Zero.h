/**
 ************************************************************************************************
 * @file       PWM_Arduino_Zero.h
 * @author     Steve Ding, Colin Gallacher
 * @version    V2.0.0
 * @date       11-December-2018
 * @brief      Extended PWM control for Haply M0 and Arduino Zero SAMD boards. offers control 
 *             for pins 0 to 13 based on Arduino pin definition
 ************************************************************************************************
 * @attention
 * 
 * This driver allows for more refined PWM controls of ATSAMD21G18A-U based haply and arduino  
 * boards on pins 0 to 13 as defined by arduino. 
 * 
 * Pins from 0 to 13 can be specified to use one of the specific 24-bit timers TCC0, TCC1 or
 * the 16-bit timer TCC2.
 * 
 * 24-bit timers (TCC0, TCC1) have max period count of 16,777,216
 * 16-bit timers (TCC2) have max period count of 65.536
 * 
 * Please refer to the table at the end of this section for timer and pin connections
 * 
 * 
 *
 * How to use this PWM driver: 
 * 
 * 
 * To setup setup a pin at a specific period and frequency, use the pwm_setup function:
 * 
 * void pwm_setup(uint32_t pwmPin, uint32_t period, uint32_t divisor, uint32_t TCCx);
 * 
 * 
 * To setup pulse width use the pwm_write_duty function:
 * 
 * void pwm_write_duty(uint32_t pwmPin, uint32_t pwmDuty);
 * 
 * 
 * Example: Setting up pin 4 to output pwm signal at 40 kHz. 
 * 
 *  In setup() section:
 *    pwm_setup(4, 1200, 1, 0);
 *    
 *    First paramter (uint32_t pwmPin):   4, the hardware pin definition.
 *    
 *    Second parameter (uint32_t period): 1200, period of the pulse width modulation. Defines 
 *                                        frequency parameter.  
 *    
 *    Third parameter (uint32_t divisor): 1, prescaler divisor for frequency adjustment. Defines
 *                                        frequency parameter. limited to 1, 2, 4, 8, 16 divisor 
 *                                        values
 *    
 *    Fourth parameter (uint32_t TCCx):   0, hardware timer specification. Pin 4 is connected to 
 *                                        TCC0 and TCC1. In this example we use TCC0
 *    
 *    The frequency is determined by the equation:
 *      frequency = GCLK / (GENDIV * PRESCALER * (Period + 1))  
 *      
 *      GCLK is linked to GCLK4 which is tied to the main clock at 48MHz
 *      GENDIV is the GCLK divisor, it is set to 1 as default
 *      PRESCALER is the prescaler divisor parameter
 *      Period is the period parameter 
 *      
 *    For our example:
 *      frequency = 48MHz / (1 * 1 * (1200 + 1))   
 *                = 40kHz
 *    
 *  To set pulse width: 
 *    pwm_write_duty(4, 300);  
 *    
 *    First parameter (uint32_t pwmPin):  4, the hardware pin 
 *    
 *    Second paramter (uint32_t pwmDuty): 300, PWM duty pulse
 *    
 *    Since our pwm signal frequency is set at 40kHz with a period of 1200, setting the pwmDuty
 *    to 300 will result in a 25% duty cycle
 *    
 *    For our example:
 *      duty_Cycle = 300/1200
 *                 = 0.25
 *  
 *  
 *  
 *  Note* pins sharing the same timer will have the same frequency with last pin setup overriding
 *        frequency settings. 
 * 
 * 
 * +------------+------------------+--------+-----------------+----------------------------------
 * + Pin number +  ZERO Board pin  |  PIN   | Label/Name      | Connected 24-bit Timers
 * +------------+------------------+--------+-----------------+----------------------------------
 * |            | Digital Low      |        |                 |
 * +------------+------------------+--------+-----------------+----------------------------------
 * | 0          | 0 -> RX          |  PA11  |                 | TCC0/WO[3]  TCC1/WO[1]
 * | 1          | 1 <- TX          |  PA10  |                 | TCC0/WO[2]  TCC1/WO[0]
 * | 2          | 2                |  PA14  |                 | TCC0/WO[4]
 * | 3          | ~3               |  PA09  |                 | TCC0/WO[1]  TCC1/WO[3]
 * | 4          | ~4               |  PA08  |                 | TCC0/WO[0]  TCC1/WO[2]
 * | 5          | ~5               |  PA15  |                 | TCC0/WO[5]
 * | 6          | ~6               |  PA20  |                 | TCC0/WO[6]
 * | 7          | 7                |  PA21  |                 | TCC0/WO[7]
 * | 8          | ~8               |  PA06  |                 | TCC1/WO[0]
 * | 9          | ~9               |  PA07  |                 | TCC1/WO[1]
 * | 10         | ~10              |  PA18  |                 | TCC0/WO[2]
 * | 11         | ~11              |  PA16  |                 | TCC2/WO[0]   TCC0/WO[6]
 * | 12         | ~12              |  PA19  |                 | TCC0/WO[3]
 * | 13         | ~13              |  PA17  | LED             | TCC2/WO[1]   TCC0/WO[7]
 * +------------+------------------+--------+-----------------+----------------------------------
 *
 ************************************************************************************************
 */

#include "Arduino.h"

#define TOTAL_PWM_PINS  14

static int TCC0_Set = 0;
static int TCC1_Set = 0;
static int TCC2_Set = 0;

static int PWM_initial_set = 0;

typedef struct pwm_pin{
  uint32_t TCCx;
}pwm_clk;

static pwm_clk pwm_pins[TOTAL_PWM_PINS];



void pwm_setup(uint32_t pwmPin, uint32_t period, uint32_t divisor, uint32_t TCCx);
void pwm_write_duty(uint32_t pwmPin, uint32_t pwmDuty);
void Timer0_Divisor(uint32_t divisor);
void Timer1_Divisor(uint32_t divisor);
void Timer2_Divisor(uint32_t divisor);
void enable_general_clock();
void link_TCC0_TCC1();
void link_TCC2();


void pwm_setup(uint32_t pwmPin, uint32_t period, uint32_t divisor, uint32_t TCCx){

  /* Enable general clock for setup */
  if(!PWM_initial_set){
    enable_general_clock();
    PWM_initial_set = 1;
  }
      
  /* link timer regesters to general clock */
  if((TCCx == 0 && TCC0_Set == 0) || (TCCx == 1 && TCC1_Set == 0)){
    link_TCC0_TCC1(); 
  }

  if(TCCx == 2 && TCC2_Set == 0){
    link_TCC2();
  }

  switch(pwmPin){
    
    case 0:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[0].ulPort].PINCFG[g_APinDescription[0].ulPin].bit.PMUXEN = 1;
      
      if(TCCx == 0){ 
        PORT->Group[g_APinDescription[0].ulPort].PMUX[g_APinDescription[0].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;

        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE); 

        REG_TCC0_PER = period;      
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC3 = 0;      
        while(TCC0->SYNCBUSY.bit.CC3);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }          
      }
      else if(TCCx == 1){
        PORT->Group[g_APinDescription[0].ulPort].PMUX[g_APinDescription[0].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
      
        REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC1->SYNCBUSY.bit.WAVE);  

        REG_TCC1_PER = period;      
        while(TCC1->SYNCBUSY.bit.PER);

        REG_TCC1_CC1 = 0;      
        while(TCC1->SYNCBUSY.bit.CC1);

        if(TCC1_Set == 0){
          Timer1_Divisor(divisor);    
        } 
      }
      break;


    case 1:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;

      if(TCCx == 0){
        PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;

        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE); 

        REG_TCC0_PER = period;      
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC2 = 0;      
        while(TCC0->SYNCBUSY.bit.CC2);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }          
      }
      else if(TCCx == 1){
        PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;
      
        REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC1->SYNCBUSY.bit.WAVE);  

        REG_TCC1_PER = period;      
        while(TCC1->SYNCBUSY.bit.PER);

        REG_TCC1_CC0 = 0;      
        while(TCC1->SYNCBUSY.bit.CC0);

        if(TCC1_Set == 0){
          Timer1_Divisor(divisor);    
        } 
      }
      break;


    case 2:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;

      if(TCCx == 0){
        PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;

        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE); 

        REG_TCC0_PER = period;      
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC0 = 0;      
        while(TCC0->SYNCBUSY.bit.CC0);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }              
      }
      break;


    case 3:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;    
      
      if(TCCx == 0){
        PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
        
        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE);  

        REG_TCC0_PER = period;      
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC1 = 0;      
        while(TCC0->SYNCBUSY.bit.CC1);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        } 
      }
      else if(TCCx == 1){
        PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;
      
        REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC1->SYNCBUSY.bit.WAVE);  

        REG_TCC1_PER = period;      
        while(TCC1->SYNCBUSY.bit.PER);

        REG_TCC1_CCB1 = 0;      
        while(TCC1->SYNCBUSY.bit.CCB1);

        if(TCC1_Set == 0){
          Timer1_Divisor(divisor);    
        }
      }
      break;


    case 4:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;    

      if(TCCx == 0){
        PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;
        
        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE);  

        REG_TCC0_PER = period;      
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC0 = 0;      
        while(TCC0->SYNCBUSY.bit.CC0);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }
      }
      else if(TCCx == 1){
        PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;
      
        REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC1->SYNCBUSY.bit.WAVE);  

        REG_TCC1_PER = period;      
        while(TCC1->SYNCBUSY.bit.PER);

        REG_TCC1_CCB0 = 0;      
        while(TCC1->SYNCBUSY.bit.CCB0);

        if(TCC1_Set == 0){
          Timer1_Divisor(divisor);    
        } 
      }
      break;


    case 5:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;    

      if(TCCx == 0){
        PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;
        
        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE);  

        REG_TCC0_PER = period;      
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC1 = 0;      
        while(TCC0->SYNCBUSY.bit.CC1);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }
      }
      break;


    case 6:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;    

      if(TCCx == 0){
        PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;
        
        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE);  

        REG_TCC0_PER = period;      
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC2 = 0;      
        while(TCC0->SYNCBUSY.bit.CC2);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }
      }
      break;


    case 7:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;    
      
      if(TCCx == 0){
        PORT->Group[g_APinDescription[7].ulPort].PMUX[g_APinDescription[7].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;
        
        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE);  

        REG_TCC0_PER = period;      
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC3 = 0;      
        while(TCC0->SYNCBUSY.bit.CC3);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }
      }
      break;


    case 8:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[8].ulPort].PINCFG[g_APinDescription[8].ulPin].bit.PMUXEN = 1;    
      
      if(TCCx == 1){
        PORT->Group[g_APinDescription[8].ulPort].PMUX[g_APinDescription[8].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;
      
        REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC1->SYNCBUSY.bit.WAVE); 

        REG_TCC1_PER = period;
        while(TCC1->SYNCBUSY.bit.PER);

        REG_TCC1_CC0 = 0;      
        while(TCC1->SYNCBUSY.bit.CC0);

        if(TCC1_Set == 0){
          Timer1_Divisor(divisor);    
        } 
      }
      break;


    case 9:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;    
      
      if(TCCx == 1){
        PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
      
        REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC1->SYNCBUSY.bit.WAVE); 

        REG_TCC1_PER = period;
        while(TCC1->SYNCBUSY.bit.PER);

        REG_TCC1_CC1 = 0;      
        while(TCC1->SYNCBUSY.bit.CC1);

        if(TCC1_Set == 0){
          Timer1_Divisor(divisor);    
        }
      }
      break;


    case 10:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1;    
      
      if(TCCx == 0){
        PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;

        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE); 

        REG_TCC0_PER = period;
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC2 = 0;      
        while(TCC0->SYNCBUSY.bit.CC2);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }        
      }
      break;


    case 11:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;    

      if(TCCx == 0){
        PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;

        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE); 

        REG_TCC0_PER = period;
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC2 = 0;      
        while(TCC0->SYNCBUSY.bit.CC2);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }        
      }
      else if(TCCx == 2){
        PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;

        REG_TCC2_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC2->SYNCBUSY.bit.WAVE); 

        REG_TCC2_PER = period;
        while(TCC2->SYNCBUSY.bit.PER);

        REG_TCC2_CC0 = 0;      
        while(TCC2->SYNCBUSY.bit.CC0);

        if(TCC2_Set == 0){
          Timer2_Divisor(divisor);    
        }
      }
      break;


    case 12:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1;    
      
      if(TCCx == 0){
        PORT->Group[g_APinDescription[12].ulPort].PMUX[g_APinDescription[12].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;  

        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE);  

        REG_TCC0_PER = period;
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC3 = 0;      
        while(TCC0->SYNCBUSY.bit.CC3);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }   
      }
      break;


    case 13:
      pwm_pins[pwmPin].TCCx = TCCx;
      PORT->Group[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;
      
      if(TCCx == 0){
        PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;

        REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC0->SYNCBUSY.bit.WAVE); 

        REG_TCC0_PER = period;
        while(TCC0->SYNCBUSY.bit.PER);

        REG_TCC0_CC3 = 0;      
        while(TCC0->SYNCBUSY.bit.CC3);

        if(TCC0_Set == 0){
          Timer0_Divisor(divisor);    
        }
      }
      else if(TCCx == 2){
        PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;

        REG_TCC2_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        
        while (TCC2->SYNCBUSY.bit.WAVE); 

        REG_TCC2_PER = period;
        while(TCC2->SYNCBUSY.bit.PER);

        REG_TCC2_CC1 = 0;      
        while(TCC2->SYNCBUSY.bit.CC1);

        if(TCC2_Set == 0){
          Timer2_Divisor(divisor);  
        }
      }
      break;

      
    default:
      break;
  }
}


void pwm_write_duty(uint32_t pwmPin, uint32_t pwmDuty){
  
  switch(pwmPin){
    case 0:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC3 = pwmDuty;
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      else if(pwm_pins[pwmPin].TCCx == 1){
        REG_TCC1_CC1 = pwmDuty;      
        while(TCC1->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 1:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC2 = pwmDuty;
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      else if(pwm_pins[pwmPin].TCCx == 1){
        REG_TCC1_CC0 = pwmDuty;      
        while(TCC1->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 2:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC0 = pwmDuty;
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 3:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC1 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      else if(pwm_pins[pwmPin].TCCx == 1){
        REG_TCC1_CCB1 = pwmDuty;      
        while(TCC1->SYNCBUSY.bit.ENABLE);
        
      }
      break;

    case 4:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC0 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      else if(pwm_pins[pwmPin].TCCx == 1){
        REG_TCC1_CCB0 = pwmDuty;      
        while(TCC1->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 5:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC1 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 6:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC2 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 7:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC3 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 8:
      if(pwm_pins[pwmPin].TCCx == 1){
        REG_TCC1_CC0 = pwmDuty;      
        while(TCC1->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 9:
      if(pwm_pins[pwmPin].TCCx == 1){
        REG_TCC1_CC1 = pwmDuty;      
        while(TCC1->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 10:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC2 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 11:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC2 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      else if(pwm_pins[pwmPin].TCCx == 2){
        REG_TCC2_CC0 = pwmDuty;      
        while(TCC2->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 12:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC3 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      break;

    case 13:
      if(pwm_pins[pwmPin].TCCx == 0){
        REG_TCC0_CC3 = pwmDuty;      
        while(TCC0->SYNCBUSY.bit.ENABLE);
      }
      else if(pwm_pins[pwmPin].TCCx == 2){
        REG_TCC2_CC1 = pwmDuty;
        while(TCC2->SYNCBUSY.bit.ENABLE);
      }
      break;
      
    default:
      break;
  }  
}


void Timer0_Divisor(uint32_t divisor){
  
  switch(divisor){
    case 2:
      REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    
                        TCC_CTRLA_ENABLE;   
      break;
    case 4:
      REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV4 |    
                        TCC_CTRLA_ENABLE;   
      break;
    case 8:
      REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    
                        TCC_CTRLA_ENABLE;   
      break;
    case 16:
      REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    
                        TCC_CTRLA_ENABLE;   
      break;
    default:
      REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    
                        TCC_CTRLA_ENABLE; 
      break;
  }   
  while (TCC0->SYNCBUSY.bit.ENABLE);
  TCC0_Set = 1; 
}


void Timer1_Divisor(uint32_t divisor){

  switch(divisor){
    case 2:
      REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    
                        TCC_CTRLA_ENABLE; 
      break;
    case 4:
      REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV4 |    
                        TCC_CTRLA_ENABLE; 
      break;
    case 8:
      REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    
                        TCC_CTRLA_ENABLE; 
      break;
    case 16:
      REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    
                        TCC_CTRLA_ENABLE; 
      break;
    default:
      REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    
                        TCC_CTRLA_ENABLE; 
      break;
  }
  while (TCC1->SYNCBUSY.bit.ENABLE);
  TCC1_Set = 1;
}


void Timer2_Divisor(uint32_t divisor){
  switch(divisor){
    case 2:
      REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    
                        TCC_CTRLA_ENABLE; 
      break;
    case 4:
      REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV4 |    
                        TCC_CTRLA_ENABLE; 
      break;
    case 8:
      REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    
                        TCC_CTRLA_ENABLE; 
      break;
    case 16:
      REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    
                        TCC_CTRLA_ENABLE; 
      break;
    default:
      REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    
                        TCC_CTRLA_ENABLE; 
      break;             
  }
  while (TCC2->SYNCBUSY.bit.ENABLE); 
  TCC2_Set = 1; 
}




void enable_general_clock(){
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization  
}


void link_TCC0_TCC1(){
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY); 
}


void link_TCC2(){
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2 and TC3
  while (GCLK->STATUS.bit.SYNCBUSY);   
}



