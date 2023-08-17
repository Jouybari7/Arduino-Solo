// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: Torque Control of BLDC equipped with HALL snesors using Arduino and SOLO
*    Author: SOLOMOTORCONTROLLERS
*    Date: 2022
*    Code version: 4.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
*    Please make sure you are applying the right wiring between SOLO and your ARDUINO
*    The Code below has been tested on Arduino MEGA
*    The Motor used for Testings: DB56C036030-A
*/

#include "SOLOMotorControllersCanopen.h" 

//For this Test, make sure you have calibrated your Motor and Hall sensors before
//to know more please read: https://www.solomotorcontrollers.com/hall-sensors-to-solo-for-controlling-speed-torque-brushless-motor/

//instanciate a SOLO object:
SOLOMotorControllers *SOLO_ObjR; 
SOLOMotorControllers *SOLO_ObjL; 


// the device address of SOLO:
// unsigned char SOLO_address1 = 0; 

//Initialize the SOLO object
int SOLOdeviceAddressR = 100; 
int SOLOdeviceAddressL = 200; 

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 20; 

//Motor's Number of Poles
long numberOfPoles = 8; 

// Current Limit of the Motor
float currentLimit = 10.0; 

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
  // int SOLOdeviceAddress = 100; 
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
   
  Serial.println("\n Both Communications Established succuessfully!");

  // Initial Configuration of the device and the Motor
  SOLO_ObjR->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_ObjR->SetCurrentLimit(currentLimit);
  SOLO_ObjR->SetMotorPolesCounts(numberOfPoles);
  SOLO_ObjR->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  SOLO_ObjR->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);
  SOLO_ObjR->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::hallSensors);
  SOLO_ObjR->SetControlMode(SOLOMotorControllers::ControlMode::torqueMode);
  
  SOLO_ObjL->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_ObjL->SetCurrentLimit(currentLimit);
  SOLO_ObjL->SetMotorPolesCounts(numberOfPoles);
  SOLO_ObjL->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  SOLO_ObjL->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);
  SOLO_ObjL->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::hallSensors);
  SOLO_ObjL->SetControlMode(SOLOMotorControllers::ControlMode::torqueMode);
  //run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Controlling
  //run ID. always after selecting the Motor Type!
  //ID. doesn't need to be called everytime, only one time after connection of a new motor it will be  enough
  //the ID. values will be remembered by SOLO after power recycling
  SOLO_ObjR->MotorParametersIdentification(SOLOMotorControllers::Action::start);
  Serial.println("\n Identifying Right Motor");

  SOLO_ObjL->MotorParametersIdentification(SOLOMotorControllers::Action::start);
  Serial.println("\n Identifying Left Motor");

  //wait at least for 2sec till ID. is done
  delay(2000); 
}

void loop() {

   //set the Direction on C.C.W. 
   SOLO_ObjR->SetMotorDirection(SOLOMotorControllers::Direction::counterclockwise);
   //set an arbitrary Positive torque reference 
   SOLO_ObjR->SetTorqueReferenceIq(7);
   // wait till motor reaches to the reference 
   delay(100);
   actualMotorTorque = SOLO_ObjR->GetQuadratureCurrentIqFeedback();
   Serial.println("\n Measured Iq/Torque[A] Right: ");
   Serial.println(actualMotorTorque);
   // wait for the motor to speed up naturally  
   delay(100);
   actualMotorSpeed = SOLO_ObjR->GetSpeedFeedback();
   Serial.println("\n Measured Speed[RPM] Right: ");
   Serial.println(actualMotorSpeed);
   // wait for the motor to speed up naturally  
   delay(1000);
   SOLO_ObjR->SetTorqueReferenceIq(0);

   //set the Direction on C.W. 
   SOLO_ObjL->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);
   //set an arbitrary Positive torque reference 
   SOLO_ObjL->SetTorqueReferenceIq(7);
   // wait till motor reaches to the reference 
   delay(100);
   actualMotorTorque = SOLO_ObjL->GetQuadratureCurrentIqFeedback();
   Serial.println("\n Measured Iq/Torque[A] Left: ");
   Serial.println(actualMotorTorque);
   // wait for the motor to speed up naturally  
   delay(100);
   actualMotorSpeed = SOLO_ObjL->GetSpeedFeedback();
   Serial.println("\n Measured Speed[RPM] Left: ");
   Serial.println(actualMotorSpeed);
   // wait for the motor to speed up naturally  
   delay(1000);
   SOLO_ObjL->SetTorqueReferenceIq(0);
   delay(1000);

  SOLO_ObjR->SetTorqueReferenceIq(4);
  SOLO_ObjL->SetTorqueReferenceIq(4);
  SOLO_ObjR->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);
  SOLO_ObjL->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);
  delay(1000);
  SOLO_ObjR->SetTorqueReferenceIq(0);
  SOLO_ObjL->SetTorqueReferenceIq(0);
  SOLO_ObjR->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);
  SOLO_ObjL->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);
  delay(1000);

  

}
