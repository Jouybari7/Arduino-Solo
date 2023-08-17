// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: Position Control of BLDC equipped with HALL snesors using Arduino and SOLO
*    Author: SOLOMOTORCONTROLLERS
*    Date: 2022
*    Code version: 4.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
*    Please make sure you are applying the right wiring between SOLO and your ARDUINO
*    The Code below has been tested on Arduino UNO
*    The Motor used for Testings: DB56C036030-A
*/

#include "SOLOMotorControllersCanopen.h" 

//For this Test, make sure you have calibrated your Motor and Hall sensors before
//to know more please read: https://www.solomotorcontrollers.com/hall-sensors-to-solo-for-controlling-speed-torque-brushless-motor/

//instanciate a SOLO object:
SOLOMotorControllers *SOLO_ObjR; 
SOLOMotorControllers *SOLO_ObjL; 

int SOLOdeviceAddressR = 100; 
int SOLOdeviceAddressL = 200; 

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 20; 

//Motor's Number of Poles
long numberOfPoles = 4; 

// Current Limit of the Motor
float currentLimit = 14.0; 

//Speed controller Kp
// float speedControllerKp = 0.1199951; 
float speedControllerKp = 0.1199951; 

//Speed controller Ki
float speedControllerKi = 0.0049972; 

//Position controller Kp
float positionControllerKp = 1000;

//Position controller Ki
float positionControllerKi = 0.001; 

// Desired Speed Limit[RPM]
long desiredSpeedLimit = 350; 

// Battery or Bus Voltage
float busVoltage = 0; 

// Motor Torque feedback
float actualMotorTorque = 0; 

// Motor speed feedback
long actualMotorSpeed = 0; 

// Motor position feedback
long actualMotorPosition = 0; 

void setup() {
  //In this example, make sure you put SOLO into Closed-Loop Mode
  
  //used for Monitoring in Arduino Only
  Serial.begin(115200); 

  //Initialize the SOLO object
  int chipSelectPin = 9; //SPI CS pin for CANshield
  SOLO_ObjR = new SOLOMotorControllersCanopen(SOLOdeviceAddressR, chipSelectPin); 
  SOLO_ObjL = new SOLOMotorControllersCanopen(SOLOdeviceAddressL, chipSelectPin); 
  delay(1000);

  Serial.println("\n Trying to Connect To SOLO");
  delay(1000);
  //wait here till communication is established
  while(SOLO_ObjR->CommunicationIsWorking() == false && SOLO_ObjL->CommunicationIsWorking() == false )
  {
    delay(500);
  }
      
  Serial.println("\n Both CommunicationS Established succuessfully!");

  // Initial Configuration of the device and the Motor
  SOLO_ObjR->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_ObjR->SetCurrentLimit(currentLimit);
  SOLO_ObjR->SetMotorPolesCounts(numberOfPoles);
  SOLO_ObjR->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  SOLO_ObjR->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);
  SOLO_ObjR->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::hallSensors);
  SOLO_ObjR->SetSpeedControllerKp(speedControllerKp);
  SOLO_ObjR->SetSpeedControllerKi(speedControllerKi);
  SOLO_ObjR->SetPositionControllerKp(positionControllerKp);
  SOLO_ObjR->SetPositionControllerKi(positionControllerKi);
  SOLO_ObjR->SetControlMode(SOLOMotorControllers::ControlMode::positionMode);

  SOLO_ObjL->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_ObjL->SetCurrentLimit(currentLimit);
  SOLO_ObjL->SetMotorPolesCounts(numberOfPoles);
  SOLO_ObjL->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  SOLO_ObjL->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);
  SOLO_ObjL->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::hallSensors);
  SOLO_ObjL->SetSpeedControllerKp(speedControllerKp);
  SOLO_ObjL->SetSpeedControllerKi(speedControllerKi);
  SOLO_ObjL->SetPositionControllerKp(positionControllerKp);
  SOLO_ObjL->SetPositionControllerKi(positionControllerKi);
  SOLO_ObjL->SetControlMode(SOLOMotorControllers::ControlMode::positionMode);
 
  //run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
  //run ID. always after selecting the Motor Type!
  //ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
  //the ID. values will be remembered by SOLO after power recycling
  SOLO_ObjR->MotorParametersIdentification(SOLOMotorControllers::Action::start);
  Serial.println("\n Identifying the Right Motor");

  SOLO_ObjL->MotorParametersIdentification(SOLOMotorControllers::Action::start);
  Serial.println("\n Identifying the Left Motor");
  //wait at least for 2sec till ID. is done
  delay(2000); 

  //set a desired Speed Limit for trajectory in RPM
  SOLO_ObjL->SetSpeedLimit(desiredSpeedLimit);
  SOLO_ObjR->SetSpeedLimit(desiredSpeedLimit);

}

void loop() {

  SOLO_ObjR->SetPositionReference(+100);
  delay(3000);
  actualMotorPosition = SOLO_ObjR->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed: Right");
  Serial.println(actualMotorPosition);

  

  SOLO_ObjL->SetPositionReference(+100);
  delay(3000); 
  actualMotorPosition = SOLO_ObjL->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed Left: ");
  Serial.println(actualMotorPosition);
  




  SOLO_ObjR->SetPositionReference(3);
  delay(3000);
  actualMotorPosition = SOLO_ObjR->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed: Right");
  Serial.println(actualMotorPosition);

  

  
  SOLO_ObjL->SetPositionReference(3);
  delay(3000); 
  actualMotorPosition = SOLO_ObjL->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed Left: ");
  Serial.println(actualMotorPosition);



  //set a desired Speed Limit for trajectory in RPM
  // set a positive desired Position Reference in terms of pulses
  SOLO_ObjR->SetPositionReference(50);
  SOLO_ObjL->SetPositionReference(50);
  delay(2000);
  actualMotorPosition = SOLO_ObjR->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed: Right");
  Serial.println(actualMotorPosition);
  actualMotorPosition = SOLO_ObjL->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed Left: ");
  Serial.println(actualMotorPosition);
}
