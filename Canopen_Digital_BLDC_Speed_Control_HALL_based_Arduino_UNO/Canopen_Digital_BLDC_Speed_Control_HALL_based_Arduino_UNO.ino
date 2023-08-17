// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: Speed Control of BLDC equipped with HALL snesors using Arduino and SOLO
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
float speedControllerKp = 0.1199951; 

//Speed controller Ki
float speedControllerKi = 0.0049972; 

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
  while(SOLO_ObjR->CommunicationIsWorking() == false && SOLO_ObjL->CommunicationIsWorking() == false)
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
  SOLO_ObjR->SetControlMode(SOLOMotorControllers::ControlMode::speedMode);

  SOLO_ObjL->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_ObjL->SetCurrentLimit(currentLimit);
  SOLO_ObjL->SetMotorPolesCounts(numberOfPoles);
  SOLO_ObjL->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  SOLO_ObjL->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);
  SOLO_ObjL->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::hallSensors);
  SOLO_ObjL->SetSpeedControllerKp(speedControllerKp);
  SOLO_ObjL->SetSpeedControllerKi(speedControllerKi);
  SOLO_ObjL->SetControlMode(SOLOMotorControllers::ControlMode::speedMode);
 
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
}

void loop() {

   //set the Direction on C.C.W. 
   SOLO_ObjR->SetMotorDirection(SOLOMotorControllers::Direction::counterclockwise);
   //set an arbitrary Positive speed reference[RPM]
   SOLO_ObjR->SetSpeedReference(350);
   // wait till motor reaches to the reference 
   delay(300);
   actualMotorSpeed = SOLO_ObjR->GetSpeedFeedback();
   Serial.println("\n Measured Speed[RPM] Right: ");
   Serial.println(actualMotorSpeed);
   delay(2000);
  SOLO_ObjR->SetSpeedReference(0);
  delay(300);



   //set the Direction on C.W. 
   SOLO_ObjL->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);
   //set an arbitrary Positive speed reference[RPM]
   SOLO_ObjL->SetSpeedReference(350);
   // wait till motor reaches to the reference 
   delay(300);
   actualMotorSpeed = SOLO_ObjL->GetSpeedFeedback();
   Serial.println("\n Measured Speed[RPM] Left: ");
   Serial.println(actualMotorSpeed);
   delay(4000);
   SOLO_ObjL->SetSpeedReference(0);

  delay(3000);

  SOLO_ObjL->SetMotorDirection(SOLOMotorControllers::Direction::counterclockwise);
  SOLO_ObjR->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);

  SOLO_ObjR->SetSpeedReference(350);
  SOLO_ObjL->SetSpeedReference(350);

  delay(3000);
  SOLO_ObjR->SetSpeedReference(0);
  SOLO_ObjL->SetSpeedReference(0);  
  delay(3000);
  
    

   
}
