/**
 ************************************************************************************************
* @file       Haply_M0_Firmware.h
* @author     Steve Ding, Colin Gallacher
* @version    V0.3.0
* @date       11-December-2018
* @brief      Haply M0 board firmware header
 ************************************************************************************************
* @attention
*
*
************************************************************************************************
*/
 

/* maximum number of ports available for on board actuator control ***************************************************/
#define TOTAL_ACTUATOR_PORTS      2
 

 

/* maximum number of available input pins ****************************************************************************/
#define PWM_PINS                  14
#define ANALOG_PINS               5
 

/* number of setup parameters per encoder ****************************************************************************/
#define ENCODER_PARAMETERS        2
 

 

/* encoder pin definitions *******************************************************************************************/
#define ENCPIN1_1                 2  // M1
#define ENCPIN1_2                 3
 

#define ENCPIN2_1                 A0 // M2
#define ENCPIN2_2                 A5
 

 

/* PWM definitions block *********************************************************************************************/
#define PWMPIN1                   9
#define DIRPIN1                   8
 

#define PWMPIN2                   12
#define DIRPIN2                   6
 

#define PWMFREQ                   40000
#define MAX_TORQUE                0.156 //3.0//0.123
#define PWM_OUTPUT_RESOLUTION     1023
 

 

/* Rotation direction definitions ************************************************************************************/
#define CW                        0
#define CCW                       1
 

 

/* actuator struct definitions ***************************************************************************************/
typedef struct motor{
  int hostAddress;
 

  int rotationalDirection;
  int pwmPin;
  int dirPin;

  float torque;
}actuator;
 

 

/* encoder struct definitions ****************************************************************************************/
typedef struct enc{
  int hostAddress;
 

  int rotationalDirection;
  float EncOffset;
  float EncResolution;
 

  Encoder *Enc;
}encoder;
 

 

/* sensor struct definitions *****************************************************************************************/
typedef struct sen{
  int hostAddress;
  int sensorPin;
}sensor;
 

/* pwm struct definitions ********************************************************************************************/
typedef struct pulseWidth{
  int hostAddress;
  int pulse;
  int pwmPin;
}pwm;

float vir_torques[4];
 

/* communication function definitions ********************************************************************************/
char command_instructions();
char reset_haply(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]);
char setup_device(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]);
char write_states(actuator actuators[]);
void read_states(encoder encoders[], sensor analog_sensors[], char device_address);
 

/* system setup command function definitions *************************************************************************/
void reset_device(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]);
void setup_sensors(sensor analog[], char sensors_active, char device_address);
void setup_pwms(pwm pulse[], char pwms_active, char device_address);
void setup_encoders(encoder encoders[], char device_address, char motors_active[], char encoder_parameters[]);
void setup_actuators(actuator actuators[], char device_address, char motors_active[]);
 

/* component initialization function definitions *********************************************************************/
void initialize_analog(sensor *analog, char sensor_port, char device_address);
void initialize_pwm(pwm *pulse, char pwm_pin, char device_address);
void initialize_encoder(encoder *enc, char device_address, char parameters[], int enc1, int enc2);
void initialize_actuator(actuator *mtr, char device_address, int pwm, int dir);
 

/* state control function definitions ********************************************************************************/
void create_torque(actuator *mtr, float torque);
void create_pulse(actuator actuators[], pwm *pin);
float read_analog_sensor(sensor *analog);
float read_digital_sensor(sensor *digital);
float read_encoder_value(encoder *enc);
 

/* parsing function definitions **************************************************************************************/
char mtr_parameters(char sequence[], actuator actuators[]);
char enc_parameters(char sequence[], encoder encoders[]);
 

/* Helper function definitions ***************************************************************************************/
void FloatToBytes(float val, char segments[]);
float BytesToFloat(char segments[]);
void ArrayCopy(char src[], int src_index, char dest[], int dest_index, int len );
 

/*   ****** Additional function for Haply project ********   */
float create_angle(actuator *mtr, float angle_value, float ref_angle);
//void create_angle2(actuator *mtr, float angle_values[], float ref_angles[], float torque_data[]);
//void forwardKinematics(float angles[], float pos[]);
 

/* communication functions *******************************************************************************************/
 

char command_instructions(){
  return SerialUSB.read();
}
 

 

char reset_haply(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]){

  char deviceAddress = SerialUSB.read();

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    actuators[i].hostAddress = 0;
    encoders[i].hostAddress = 0;
  }
 

  for(int i = 0; i < PWM_PINS; i++){
    pulse[i].hostAddress = 0;
    pulse[i].pulse = 0;
    //digitalWrite(pulse[i].pwmPin, pulse[i].pulse);
    //analogWrite(pulse[i].pwmPin, pulse[i].pulse);
    //pwm_write_duty(pulse[i].pwmPin, pulse[i].pulse);
  }

  for(int i = 0; i < ANALOG_PINS; i++){
    analog[i].hostAddress = 0;
  }

  return deviceAddress;
}
 

 

char setup_device(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]){

  reset_device(actuators, encoders, analog, pulse);

  char motorsActive[4];
  char encodersActive[4];
  //pwm_set_resolution(12);

  char deviceAddress = SerialUSB.read();
  char motorNumbers = mtr_parameters(motorsActive, actuators);
  char encoderNumbers = enc_parameters(encodersActive, encoders);

  char sensorsActive = SerialUSB.read();
  setup_sensors(analog, sensorsActive, deviceAddress);

  byte pwmsActive = SerialUSB.read();
  setup_pwms(pulse, pwmsActive, deviceAddress);

  if(encoderNumbers > 0){
    char encoderParameters[encoderNumbers * ENCODER_PARAMETERS * 4];

    SerialUSB.readBytes(encoderParameters, encoderNumbers * ENCODER_PARAMETERS * 4);
    setup_encoders(encoders, deviceAddress, encodersActive, encoderParameters);
  }

  setup_actuators(actuators, deviceAddress, motorsActive);

  return deviceAddress;
}


char write_states(pwm pwmPins[], actuator actuators[]){

  int dataLength = 0;
  char segments[4];
  char deviceAddress = SerialUSB.read();

  for(int i = 0; i < PWM_PINS; i++){
    if(pwmPins[i].hostAddress == deviceAddress){
      pwmPins[i].pulse = SerialUSB.read();
      create_pulse(actuators, &pwmPins[i]);
    }
  }

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    if(actuators[i].hostAddress == deviceAddress){
      dataLength++;
    }
  }

  char torqueValues[4 * dataLength];
  SerialUSB.readBytes(torqueValues, 4 * dataLength);

  int j = 0;

  uint8_t scaleTorques = 0;
  float incomingTorque = 0.0;
  float detectedMaxTorque = MAX_TORQUE;
  float torque_temp = 0.0;

  // scaling code
  // read states
  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){

    if(actuators[i].hostAddress == deviceAddress){
      ArrayCopy(torqueValues, j, segments, 0, 4);
      j = j + 4;
      
      // this torrque data only refrect processing torque data
      // you need to calculate the local torque data and remote torque data

      torque_temp = actuators[i].torque;
      vir_torques[i] = BytesToFloat(segments);
      
      actuators[i].torque = torque_temp;//+vir_torques[i];
      incomingTorque = abs(actuators[i].torque);
          
      if(incomingTorque > detectedMaxTorque){
        detectedMaxTorque = incomingTorque;
        scaleTorques = 1;
      }
    }
  }

  // produce torque
  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    if(actuators[i].hostAddress == deviceAddress){

      if(scaleTorques){
        actuators[i].torque = (MAX_TORQUE / detectedMaxTorque) * actuators[i].torque; 
      }
 
      create_torque(&actuators[i], actuators[i].torque);
    }
  }

  return deviceAddress;
}
 

 

void read_states(encoder encoders[], sensor analogSensors[], char deviceAddress){
  int dataLength = 0;
  float value;
  char segments[4];
 

  for(int i = 0; i < ANALOG_PINS; i++){
    if(analogSensors[i].hostAddress == deviceAddress){
      dataLength++;
    }
  }
   

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    if(encoders[i].hostAddress == deviceAddress){
      dataLength++;
    }
  }

  char stateValues[4 * dataLength + 1];
  stateValues[0] = deviceAddress;

  int j = 1;
  for(int i = 0; i < ANALOG_PINS; i++){
    if(analogSensors[i].hostAddress == deviceAddress){
      value = read_analog_sensor(&analogSensors[i]);
      FloatToBytes(value, segments);
      ArrayCopy(segments, 0, stateValues, j, 4);
      j = j + 4;
    }
  }
 

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    if(encoders[i].hostAddress == deviceAddress){
      value = read_encoder_value(&encoders[i]);
      FloatToBytes(value, segments);
      ArrayCopy(segments, 0, stateValues, j, 4);
      j = j + 4;
    }
  }
 

  SerialUSB.write(stateValues, 4 * dataLength + 1);
}
 

/* system setup command functions ************************************************************************************/
 

void reset_device(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]){

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    actuators[i].hostAddress = 0;
    encoders[i].hostAddress = 0;
  }

  for(int i = 0; i < PWM_PINS; i++){
    pulse[i].hostAddress = 0;
    pulse[i].pulse = 0;
    //digitalWrite(pulse[i].pwmPin, pulse[i].pulse);
    //analogWrite(pulse[i].pwmPin, pulse[i].pulse);
    //pwm_write_duty(pulse[i].pwmPin, pulse[i].pulse);
  }

  for(int i = 0; i < ANALOG_PINS; i++){
    analog[i].hostAddress = 0;
  }
}
 

void setup_sensors(sensor analog[], char sensorsActive, char deviceAddress){
    char sensorPort;
    
    for(int i = 0; i < sensorsActive; i++){
      sensorPort = SerialUSB.read();

      // analog input on M0 is from 14 to 19, mapping A0 - A5, hard remap from M3 where analog in starts at 54
      sensorPort = sensorPort - 40;

      if((sensorPort >= 14) && (sensorPort <= 19)){ 
        initialize_analog(&analog[sensorPort-14], sensorPort, deviceAddress);
      }
    }
}

void setup_pwms(pwm pulse[], char pwmsActive, char deviceAddress){
  byte pwmPin;
  
  for(int i = 0; i < pwmsActive; i++){
    pwmPin = SerialUSB.read();

    if((pwmPin >= 0) && (pwmPin <= 13)){
      initialize_pwm(&pulse[pwmPin], pwmPin, deviceAddress);
    }
  }
}

void setup_encoders(encoder encoders[], char deviceAddress, char encodersActive[], char encoderParameters[]){
  char segments[4 * ENCODER_PARAMETERS];
  int j = 0;

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    if(encodersActive[i] > 0){
      switch(i){
        case 0:
          ArrayCopy(encoderParameters, j, segments, 0, 4*ENCODER_PARAMETERS);
          initialize_encoder(&encoders[i], deviceAddress, segments, ENCPIN1_1, ENCPIN1_2);
          j = j + 4*ENCODER_PARAMETERS;  
          break;   
        case 1:
          ArrayCopy(encoderParameters, j, segments, 0, 4*ENCODER_PARAMETERS);
          initialize_encoder(&encoders[i], deviceAddress, segments, ENCPIN2_1, ENCPIN2_2);
          j = j + 4*ENCODER_PARAMETERS;
          break;
      }
    }
  }
}
 

void setup_actuators(actuator actuators[], char deviceAddress, char motorsActive[]){
 int j = 0;
 
  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    if(motorsActive[i] > 0){
      switch(i){
        case 0:
          initialize_actuator(&actuators[i], deviceAddress, PWMPIN1, DIRPIN1);
          j++;
          break;
        case 1:
          initialize_actuator(&actuators[i], deviceAddress, PWMPIN2, DIRPIN2);
          j++;
          break;
      }
    }
  }
}

/* component initialization functions ********************************************************************************/
 

void initialize_analog(sensor *analog, char sensorPort, char deviceAddress){
  analog->hostAddress = deviceAddress;
  analog->sensorPin = sensorPort;

  // only setup pin if pin are A1 and A2 (pins 15 and 16) on Haply M0 since those are the only ones exposed
  if((analog->sensorPin == 15) || (analog->sensorPin == 16)){
    pinMode(analog->sensorPin, INPUT);
  }
}

 

void initialize_pwm(pwm *pulse, char pwmPin, char deviceAddress){
  pulse->hostAddress = deviceAddress;
  pulse->pwmPin = pwmPin;

//  if(pulse->pwmPin == 0 || pulse->pwmPin == 1 || pulse->pwmPin == 13){
//    pinMode(pulse->pwmPin, OUTPUT);
//  }
 
  switch(pulse->pwmPin){
    case 0:
      pinMode(pulse->pwmPin, OUTPUT);
      pwm_setup(pulse->pwmPin, PWM_OUTPUT_RESOLUTION, 1, 1);
      break;
    case 1:
      pinMode(pulse->pwmPin, OUTPUT);
      pwm_setup(pulse->pwmPin, PWM_OUTPUT_RESOLUTION, 1, 1);
      break;
    case 4:
      pinMode(pulse->pwmPin, OUTPUT);
      pwm_setup(pulse->pwmPin, PWM_OUTPUT_RESOLUTION, 1, 1);
      break;
    case 13:
      pinMode(pulse->pwmPin, OUTPUT);
      pwm_setup(pulse->pwmPin, 60000, 16, 2);
      break;
  }
}
 

 

void initialize_encoder(encoder *enc, char deviceAddress, char parameters[], int enc1, int enc2){
  int i = 0;
  char segments[4];
  enc->hostAddress = deviceAddress;
  ArrayCopy(parameters, i, segments, 0, 4);
  enc->EncOffset = BytesToFloat(segments);
  i = i + 4;
  // encoder driver does not function as quadrature, only detects on half the rate
  ArrayCopy(parameters, i, segments, 0, 4);
  enc->EncResolution = BytesToFloat(segments)/2;
  if(enc->rotationalDirection > 0){
    int temp = enc1;
    enc1 = enc2;
    enc2 = temp;
  }
  enc->Enc = new Encoder(enc1, enc2);
  enc->Enc->write(enc->EncOffset * enc->EncResolution / 360);
}
 

 

void initialize_actuator(actuator *mtr, char deviceAddress, int pwm, int dir){
  mtr->hostAddress = deviceAddress;
  mtr->pwmPin = pwm;
  mtr->dirPin = dir;
 
  pinMode(mtr->pwmPin, OUTPUT);
  pinMode(mtr->dirPin, OUTPUT);

  //pwm_setup(mtr->pwmPin, PWMFREQ, 1);
  // setups pwm pins with a period/resolution of 1023 ~ 47kHz
  // pin 9 uses timer 1
  // pin 12 uses timer 0
  switch(mtr->pwmPin){
    case 9:
     pwm_setup(mtr->pwmPin, PWM_OUTPUT_RESOLUTION, 1, 1);
      break;
    case 12:
      pwm_setup(mtr->pwmPin, PWM_OUTPUT_RESOLUTION, 1, 0);
      break;
  }
}
 

// additional function for Haply project
// @param *mtr :
// @param angle_value: actual motor angle (encoder value)
// @param ref_angle: target motor angle
float create_angle(actuator *mtr, float angle_value, float ref_angle){
  bool status = false;
  float M1_ref_angle = 0.0;
  float M2_ref_angle = 0.0;
  float angle_res=0.0;
  float res = 0.0;
  int Motor_duty=0;
  float M_torque = 0.0;

  // calculate duty corresponds to the angle difference
    switch(mtr->pwmPin){
    case 9:
      M1_ref_angle = ref_angle;
      angle_res = (M1_ref_angle - angle_value);
      res = (M1_ref_angle - angle_value)/2.0;
      Motor_duty = (-1 / (res*res+1) +1 ) * 800.0;

      M_torque = MAX_TORQUE *(float)(Motor_duty/1023.0);
      if(angle_res >= 0.0){
        mtr->torque=M_torque;
      }
      else{
        mtr->torque=-M_torque;
      }

      break;
    case 12:
      M2_ref_angle = ref_angle;
      angle_res = (M2_ref_angle - angle_value);
      res = (M2_ref_angle - angle_value)/2.0;
      Motor_duty = (-1 / (res*res+1) +1 ) * 800.0;
      M_torque = MAX_TORQUE *(Motor_duty/1023.0);
      if(angle_res >= 0.0){
        mtr->torque=M_torque;
      }
      else{
        mtr->torque=-M_torque;
      }
      break;
    default:
      break;
  }
  return M_torque;
}


//// additional function for Haply project
//// @param *mtr :
//// @param angle_value: actual motor angle (encoder value)
//// @param ref_angle: target motor angle
//void create_angle2(actuator *mtr, float angle_value[], float ref_angles[], float prev_angle[], float torque_data[]){
//
//  float M1_ref_angle = 0.0;
//  float M2_ref_angle = 0.0;
//  float angle_res=0.0;
//  float res = 0.0;
//  int Motor_duty=0;
//  float diff_angle = 0.0;
//
////  float prev_angle;
//
//  ref_angles[0] = -30.0;
//
//
//     // PD control
//    float P = 0.03;
//    float D = 0.01;
//    float smoothing = 0.8;
//
//    float angle_diff = (ref_angles[0] - angle_value[0]);
//    
//    float buff_angle = (angle_diff - prev_angle[0])/0.05;
//
//    diff_angle = smoothing*diff_angle + (1.0 - smoothing)*buff_angle; 
//
//    torque_data[0] = 0.0;//P*angle_diff;// + D*diff_angle;
//    torque_data[1] = 0.0; 
//
//    prev_angle[0] = angle_value[0];
//    prev_angle[1] = 0.0;
//
//}

/* state control functions *******************************************************************************************/
 

void create_torque(actuator *mtr, float torque){
 

  int duty;
  bool output = LOW;
 

  if(mtr->rotationalDirection == 0){
    output = !output;
  }
 

  if(torque <= 0){
    digitalWrite(mtr->dirPin, output);
  }
  else{
    digitalWrite(mtr->dirPin, !output);
  }
 

  torque = abs(torque);
 

  if(torque > MAX_TORQUE){
    torque = MAX_TORQUE;
  }
 
  duty = PWM_OUTPUT_RESOLUTION * torque / MAX_TORQUE;
  
  pwm_write_duty(mtr->pwmPin, duty);
}
 

 

void create_pulse(actuator actuators[], pwm *pin){
//  int check = 1;
//
//  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
//    if(actuators[i].pwmPin == pin->pwmPin){
//      check = 0;
//    }
//  }
//
//  if(check){
//    analogWrite(pin->pwmPin, pin->pulse);
//  }
 

  int duty;
 

  switch(pin->pwmPin){
    case 0:
      duty = pin->pulse * 1023 / 255;
      pwm_write_duty(pin->pwmPin, duty);
      break;
    case 1:
      duty = pin->pulse * 1023 / 255;
      pwm_write_duty(pin->pwmPin, duty);
      break;
    case 4:
      duty = pin->pulse * 1023 / 255;
      pwm_write_duty(pin->pwmPin, duty);
      break;
    case 13:
      duty = pin->pulse * 60000 / 255;
      pwm_write_duty(pin->pwmPin, duty);
      break;
  }
 

}
 

 

float read_analog_sensor(sensor *analog){
  float value;
 

  if(analog->sensorPin == 15 || analog->sensorPin == 16){
    value = analogRead(analog->sensorPin);
  }
  else{
    value = 0;
  }
 

  return value;
}
 
float read_encoder_value(encoder *enc){
  float thDegrees;
  thDegrees = 360.0 * enc->Enc->read()/enc->EncResolution;
  //thDegrees = 360.0 * enc->pos/enc->EncResolution;
 

  return thDegrees;
}
 

 

 

/* parsing functions *************************************************************************************************/
 

char mtr_parameters(char sequence[], actuator actuators[]){
 

  char motorUse = SerialUSB.read();
 

  int motorCount = 0;
 

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    sequence[i] = motorUse & 0x01;
    motorUse = motorUse >> 1;
   

    if(sequence[i] == 1){
      motorCount++;
    }
  }
 

  for(int i = 0; i < motorCount; i++){
    actuators[i].rotationalDirection = SerialUSB.read();
  }
 

  return (char)motorCount;
}
 

 

char enc_parameters(char sequence[], encoder encoders[]){
 

  char encoderUse = SerialUSB.read();
 

  int encoderCount = 0;
 

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    sequence[i] = encoderUse & 0x01;
    encoderUse = encoderUse >> 1;
 

    if(sequence[i] == 1){
      encoderCount++;
    }
  }
 

  for(int i = 0; i < encoderCount; i++){
    encoders[i].rotationalDirection = SerialUSB.read();
  }
 

  return (char)encoderCount;
}
 

/* Helper functions **************************************************************************************************/
 

/**
* Union definition for floating point and integer representation conversion
*/
typedef union{
  long val_l;
  float val_f;
} ufloat;
 

/**
* Translates a 32-bit floating point into an array of four bytes
*
 * @note     None
* @param    val: 32-bit floating point
* @param    segments: array of four bytes
* @return   None
 */
void FloatToBytes(float val, char segments[]){
  ufloat temp;
 

  temp.val_f = val;
 

  segments[3] = (char)((temp.val_l >> 24) & 0xff);
  segments[2] = (char)((temp.val_l >> 16) & 0xff);
  segments[1] = (char)((temp.val_l >> 8) & 0xff);
  segments[0] = (char)((temp.val_l) & 0xff);
}
 

 

/**
* Translates an array of four chars into a floating point
*
 * @note     None
* @param    segment: the input array of four bytes
* @return   Translated 32-bit floating point
 */
float BytesToFloat(char segments[]){
  ufloat temp;
 

  temp.val_l = (temp.val_l | (segments[3] & 0xff)) << 8;
  temp.val_l = (temp.val_l | (segments[2] & 0xff)) << 8;
  temp.val_l = (temp.val_l | (segments[1] & 0xff)) << 8;
  temp.val_l = (temp.val_l | (segments[0] & 0xff));
 

  return temp.val_f;
}
 

 

/**
* Copies elements from one array to another
*
 * @note     None
* @param    src: The source array to be copied from
* @param    src_index: The starting index of the source array
* @param    dest: The destination array to be copied to
* @param    dest_index: The starting index of the destination array
* @param    len: Number of elements to be copied
* @return   None
 */
void ArrayCopy(char src[], int src_index, char dest[], int dest_index, int len ){
  for(int i = 0; i < len; i++){
    dest[dest_index + i] = src[src_index + i];
  }
}
 
