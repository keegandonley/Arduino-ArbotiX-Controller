

//EE485/CES592 PROJECT
//DO NOTE CHANGE ANYTHING HERE
//GO TO LINE 75

#include <ax12.h>
#include <BioloidController.h>
#include <math.h>
#include "poses.h"

BioloidController bioloid = BioloidController(1000000);

const int SERVOCOUNT = 8;
int id;
int pos;
int pos2;
int x;
int y;
int z;
int theta1;
int theta2;
int theta3;
int theta4;
int theta5;
int jointpos1;
int jointpos2;
int jointpos3;
int jointpos4;
int jointpos5;
int currentjointpos1 = 512;
int currentjointpos2 = 512;
int currentjointpos3 = 512;
int currentjointpos4 = 512;
int currentjointpos5 = 512;
int currentjointpos6 = 512;
int currentjointpos7 = 512;
int currentjointpos8 = 512;

const int servo1 = 1;
const int servo2 = 2;
const int servo3 = 3;
const int servo4 = 4;
const int servo5 = 5;
const int servo6 = 6;
const int servo7 = 7;
const int servo8 = 8;
boolean IDCheck;
boolean RunCheck;

void setup(){
  pinMode(0,OUTPUT);  

  //initialize variables ++ 
  id = 1;
  pos = 0;
  IDCheck = 1;
  RunCheck = 0;
  //open serial port
  Serial.begin(9600);
  delay (500);   
  Serial.println("###########################");    
  Serial.println("Serial Communication Established.");    
  //Check Lipo Battery Voltage
  CheckVoltage();

  // MoveCenter(); 
  // delay(5000);

  //Scan Servos, return position.
  ScanServo();

  // MoveTest();

  MoveHome();

  MenuOptions();

  RunCheck = 1;
}

void loop(){
  // read the sensor:

  int inByte = Serial.read();

  switch (inByte) {

  case '1':    
    ScanServo();
    break;

  case '2':    
    MoveCenter();
    MoveTest();
    break;

  case '3':    
    MoveHome();
    break;     

  case '5':    
    Project1();
    break;     

  }


}

//****************************************
//Project 1 Code
//Do not change anything above this line

void Project1(){
  Serial.println("###########################");
  Serial.println("Running Project 1 Commands");
  Serial.println("###########################");
  delay (1000);


  //Assign x,y,z
  x = 10;
  y = 10;
  z = 10;
  
  const int S = z;
  const int R = x;
  
  const int A = 15;
  const int B = 15.5;
  const int C = 12;


  Serial.println("Moving to (X, Y, Z)");
  Serial.print("X: ");
  Serial.println(x);
  Serial.print("Y: ");
  Serial.println(y);
  Serial.print("Z: ");
  Serial.println(z);
  delay (1000);

  //Given x, y, z find the angles for joint 1 - 5 (theta1 - theta5)
  int e = sqrt((pow((R - C), 2) + pow(S, 2)));
  theta1 = (atan(x / y));
  theta2 = (asin((S - B) / A));
  theta3 = (atan(e / B));
  theta4 = (atan(C / B));
  theta5 = 0;

  //Convert theta1-theta5 to jointpos1-jointpos5
  jointpos1 = theta1 * 20;
  jointpos2 = theta2 * 20;
  jointpos3 = theta3 * 20;
  jointpos4 = theta4 * 20;
  jointpos5 = theta5 * 20;

  //move Servo 1 - Servo 7  
  //jointpos1 -> Servo ID: 1
  //jointpos2 -> Servo ID: 2 & 3  IMP You need to move both servos 2 & 3 simultaneously and in opposite directions!!!
  //jointpos3 -> Servo ID: 4 & 5  IMP You need to move both servos 4 & 5 simultaneously and in opposite directions!!!
  //jointpos4 -> Servo ID: 6
  //jointpos5 -> Servo ID: 7

  // Moving joint1, that is Servo 1
  currentjointpos1 =  ax12GetRegister(servo1, 36, 2);
  Serial.println("Current Joint1 Position:");
  Serial.println(currentjointpos1);
  Serial.println("End Joint1 Position:");
  Serial.println(jointpos1);
  delay(1000);
  if (currentjointpos1 > jointpos1)
  {
    while(currentjointpos1 >= jointpos1) 
    {
      SetPosition(servo1, currentjointpos1);
      currentjointpos1 = currentjointpos1--;
      delay(20);
    }
  }
  else
  {
    while(currentjointpos1 <= jointpos1) 
    {
      SetPosition(servo1, currentjointpos1);
      currentjointpos1 = currentjointpos1++;
      delay(20);
    }
  }

  // Moving joint3, that is Servos 4 & 5 (simultaneiously, in the oppossite directions)   
  currentjointpos4 =  ax12GetRegister(servo4, 36, 2);
  currentjointpos5 =  ax12GetRegister(servo5, 36, 2);
  Serial.println("Current Joint3 Position:");
  Serial.println(currentjointpos4);
  Serial.println("End Joint3 Position:");
  Serial.println(jointpos3);
  delay(1000);
  if (currentjointpos4 > jointpos3)
  {
    while(currentjointpos4 >= jointpos3) 
    {
      SetPosition(servo4, currentjointpos4); 
      SetPosition(servo5, currentjointpos5);
      currentjointpos4 = currentjointpos4--;
      currentjointpos5 = currentjointpos5++;
      delay(20);
    }
  }
  else
  {
    while(currentjointpos4 <= jointpos3) 
    {
      SetPosition(servo4, currentjointpos4); 
      SetPosition(servo5, currentjointpos5);
      currentjointpos4 = currentjointpos4++;
      currentjointpos5 = currentjointpos5--;
      delay(20);
    }
  }



  //End your code here
  //Do not change anything below this line
  //********************************************


  if (RunCheck == 1){
    MenuOptions();
  }
}






void ScanServo(){
  id = 1;  
  Serial.println("###########################");
  Serial.println("Starting Servo Scanning Test.");
  Serial.println("###########################");

  while (id <= SERVOCOUNT){
    pos =  ax12GetRegister(id, 36, 2);
    Serial.print("Servo ID: ");
    Serial.println(id);
    Serial.print("Servo Position: ");
    Serial.println(pos);

    if (pos <= 0){
      Serial.println("###########################");
      Serial.print("ERROR! Servo ID: ");
      Serial.print(id);
      Serial.println(" not found. Please check connection and verify correct ID is set.");
      Serial.println("###########################"); 
      IDCheck = 0;
    }

    id = (id++)%SERVOCOUNT;
    delay(100);
  }
  if (IDCheck == 0){
    Serial.println("###########################");
    Serial.println("ERROR! Servo ID(s) are missing from Scan. Please check connection and verify correct ID is set.");
    Serial.println("###########################");  
  }
  else{
    Serial.println("All servo IDs present.");
  }
  if (RunCheck == 1){
    MenuOptions();
  }

}




void CheckVoltage(){  
  // wait, then check the voltage (LiPO safety)
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.println("###########################");   
  Serial.print ("System Voltage: ");
  Serial.print (voltage);
  Serial.println (" volts.");
  if (voltage < 10.0){
    Serial.println("Voltage levels below 10v, please charge battery.");
    while(1);
  }  
  if (voltage > 10.0){
    Serial.println("Voltage levels nominal.");
  }
  if (RunCheck == 1){
    MenuOptions();
  }
  Serial.println("###########################"); 
}

void MoveCenter(){
  delay(100);                    // recommended pause
  bioloid.loadPose(Center);   // load the pose from FLASH, into the nextPose buffer
  bioloid.readPose();            // read in current servo positions to the curPose buffer
  Serial.println("###########################");
  Serial.println("Moving servos to centered position");
  Serial.println("###########################");    
  delay(1000);
  bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
  while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
    bioloid.interpolateStep();     // move servos, if necessary. 
    delay(3);
  }
  if (RunCheck == 1){
    MenuOptions();
  }
}


void MoveHome(){
  delay(100);                    // recommended pause
  bioloid.loadPose(Home);   // load the pose from FLASH, into the nextPose buffer
  bioloid.readPose();            // read in current servo positions to the curPose buffer
  Serial.println("###########################");
  Serial.println("Moving servos to Home position");
  Serial.println("###########################");    
  delay(1000);
  bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
  while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
    bioloid.interpolateStep();     // move servos, if necessary. 
    delay(3);
  }
  if (RunCheck == 1){
    MenuOptions();
  }
}




void MoveTest(){
  Serial.println("###########################");
  Serial.println("Initializing Movement Sign Test");  
  Serial.println("###########################");
  delay(500);  
  id = 1;
  pos = pos2 = 512;

  // Base Servo Test

  Serial.println("Moving Servo ID: 1");

  while(pos >= 312){  
    SetPosition(1, pos);
    pos = pos--;
    delay(10);
  }

  while(pos <= 512){  
    SetPosition(1, pos);
    pos = pos++;
    delay(10);
  }

  delay(500);

  // Shoulder Servos Test  

  Serial.println("Moving Servo IDs: 2 & 3 (Shoulder)"); 
  while(pos >= 312){  
    SetPosition(2, pos);
    SetPosition(3, pos2);
    pos = pos--;
    pos2 = pos2++;
    delay(10);
  }

  while(pos <= 512){  
    SetPosition(2, pos);
    SetPosition(3, pos2);
    pos = pos++;
    pos2 = pos2--;
    delay(10);
  }

  delay(500);

  // Elbow Servo Test  

  Serial.println("Moving Servo IDs: 4 & 5 (Elbow)"); 
  while(pos <= 712){  
    SetPosition(4, pos);
    SetPosition(5, pos2);
    pos = pos++;
    pos2 = pos2--;
    delay(10);
  }

  while(pos >= 512){  
    SetPosition(4, pos);
    SetPosition(5, pos2);
    pos = pos--;
    pos2 = pos2++;
    delay(10);
  }

  delay(500);  

  //Wrist Servo Test

  Serial.println("Moving Servo ID: 6");

  while(pos <= 712){  
    SetPosition(6, pos);
    pos = pos++;
    delay(10);
  }

  while(pos >= 512){  
    SetPosition(6, pos);
    pos = pos--;
    delay(10);
  }

  delay(500);   

  //Wrist Rotate Servo Test  

  Serial.println("Moving Servo ID: 7");

  while(pos >= 312){  
    SetPosition(7, pos);
    pos = pos--;
    delay(10);
  }

  while(pos <= 512){  
    SetPosition(7, pos);
    pos = pos++;
    delay(10);
  }

  delay(500);   

  //Gripper Servo Test  

  Serial.println("Moving Servo ID: 8");

  while(pos >= 312){  
    SetPosition(8, pos);
    pos = pos--;
    delay(10);
  }

  while(pos <= 512){  
    SetPosition(8, pos);
    pos = pos++;
    delay(10);
  }

  delay(500);   

  if (RunCheck == 1){
    MenuOptions();
  }

}



void MenuOptions(){

  Serial.println("###########################"); 
  Serial.println("Please enter option 1-3 to run individual tests again.");
  Serial.println("1) Servo Scanning Test");        
  Serial.println("2) Perform Movement Sign Test");
  Serial.println("3) Move to Home Position");    
  Serial.println("5) Project-1");      
  Serial.println("###########################"); 
}


void RelaxServos(){
  id = 1;
  Serial.println("###########################");
  Serial.println("Relaxing Servos.");
  Serial.println("###########################");    
  while(id <= SERVOCOUNT){
    Relax(id);
    id = (id++)%SERVOCOUNT;
    delay(50);
  }
  if (RunCheck == 1){
    MenuOptions();
  }
}

void LEDTest(){
  id = 1;
  Serial.println("###########################");
  Serial.println("Running LED Test");
  Serial.println("###########################");    
  while(id <= SERVOCOUNT){
    ax12SetRegister(id, 25, 1);
    Serial.print("LED ON - Servo ID: ");
    Serial.println(id);
    delay(3000);
    ax12SetRegister(id, 25, 0);  
    Serial.print("LED OFF - Servo ID: ");
    Serial.println(id);    
    delay(3000);    
    id = id++;
  }

  if (RunCheck == 1){
    MenuOptions();
  }
}


