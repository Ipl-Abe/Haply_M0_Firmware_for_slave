/**
 ************************************************************************************************
 * @file       Haply_M0_Firmware.ino
 * @author     Steve Ding, Colin Gallacher
 * @version    V0.3.0
 * @date       11-December-2018
 * @brief      Haply M0 board firmware for encoder and sensor read and torque write using 
 *             on-board actuator ports
 ************************************************************************************************
 * @attention
 *
 *
 ************************************************************************************************
 */

#include <stdlib.h>
#include <Encoder.h>
#include "ADC_Boost.h"
#include "PWM_Arduino_Zero.h"
#include "Haply_M0_Firmware_V04.h"

char torque_data[4];
char M1_angle_data[4];
char M2_angle_data[4];

char M1_torque_data[4];
char M2_torque_data[4];
int count=0;

bool M1_angle_status = false;
bool M2_angle_status = false;

/* Actuator, Encoder, Sensors parameter declarations *******************************************/
actuator actuators[TOTAL_ACTUATOR_PORTS];
encoder encoders[TOTAL_ACTUATOR_PORTS];
pwm pwmPins[PWM_PINS];
sensor analogSensors[ANALOG_PINS];


/* Actuator Status and Command declarations ****************************************************/

/* Address of device that sent data */
char deviceAddress;

/* communication interface control, defines type of instructions recieved */
char cmdCode;

/* communication interface control, defines response to send */
char replyCode = 3;

/* Iterator and debug definitions **************************************************************/
long lastPublished = 0;
long currentState = 0;


/* main setup and loop block  *****************************************************************/

/**
 * Main setup function, defines parameters and hardware setup
 */
void setup() {
  ADC_Boost();
  SerialUSB.begin(0);
//  Serial1.begin(97600);  // Do not work
//  Serial1.begin(115200); // Ok but unstable
//  Serial1.begin(230400); // Seems ok
    Serial1.begin(460800); // Stable (maybe most)
  //Serial1.begin(800000); // Stable
//  Serial1.begin(921600);   // Do not work(2*460800)

}


/**
 * Main loop function
 */
void loop() {
  int cmdCode2;
  float fda;
  float M1_torque_data=0.0;
  float M2_torque_data=0.0;
  float M1_angle_value=0;
  float M2_angle_value=0;

  float M1_ref_angle=0;
  float M2_ref_angle=0;
  char M1_angle_char[4];
  char M2_angle_char[4];

    
  float M1_ref_torque=0;
  float M2_ref_torque=0;
  char M1_torque_char[4];
  char M2_torque_char[4];
  char inChar;
  
  currentState = micros();
  
  if(currentState - lastPublished >= 50){
    
    lastPublished = currentState;

    if(SerialUSB.available() > 0){

      cmdCode = command_instructions();

      
      switch(cmdCode){
        case 0:
          deviceAddress = reset_haply(actuators, encoders, analogSensors, pwmPins);
          break;
        case 1:
          deviceAddress = setup_device(actuators, encoders, analogSensors, pwmPins);
          break;
        case 2:
          deviceAddress = write_states(pwmPins, actuators);
          replyCode = 1;

          // get current status from encoders       
          M1_angle_value = read_encoder_value(&encoders[0]);
          FloatToBytes(M1_angle_value, M1_angle_data);
          M2_angle_value = read_encoder_value(&encoders[1]);
          FloatToBytes(M2_angle_value, M2_angle_data);
          
          FloatToBytes(actuators[0].torque, M1_torque_char);
          FloatToBytes(actuators[1].torque, M2_torque_char);
          
          // rend torque data to master Haply device
          if(count%1 == 0){
            Serial1.write('+');
            Serial1.write(M1_torque_char, 4);
            Serial1.write(M2_torque_char, 4);
          }

          // recieve angle data from master Haply device
          inChar = (char)Serial1.read();
          if(inChar == '+'){
            Serial1.readBytes(M1_angle_char,4);
            Serial1.readBytes(M2_angle_char,4);
            if(M1_angle_status && M2_angle_status){
              M1_ref_angle= BytesToFloat(M1_angle_char);
              M2_ref_angle= BytesToFloat(M2_angle_char);
            }
            M1_angle_status = create_angle(&actuators[0],  M1_angle_value, M1_ref_angle );
            M2_angle_status = create_angle(&actuators[1],  M2_angle_value, M2_ref_angle );
          }

          count++;
            break;
          default:
            break;
        }
    }


    switch(replyCode){
      case 0:
        break;
      case 1:
        read_states(encoders, analogSensors, deviceAddress);
        replyCode = 3;
        break;
      default:
        break;
    }
  }

}
