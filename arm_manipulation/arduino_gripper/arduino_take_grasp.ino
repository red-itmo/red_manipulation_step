#include <ros.h>
#include <std_srvs/Empty.h>
#include <DynamixelSerial1.h>


ros::NodeHandle  nh;
int encTickToDeg = 1024/360, motorSpeed = 200, offsetZero14 = 200, offsetZero1 = 30;
bool grasp(const std_srvs::Empty::Request & req, const std_srvs::Empty::Response & res){
  Dynamixel.ledStatus(1,OFF);
  delay(20);
  Dynamixel.moveSpeed(14,45*encTickToDeg+offsetZero14,motorSpeed);
  delay(20);
  Dynamixel.moveSpeed(1,45*encTickToDeg+offsetZero1,motorSpeed);  
  delay(20);
  Dynamixel.ledStatus(1,ON);  
  return true;
}

bool release_arm(const std_srvs::Empty::Request & req, const std_srvs::Empty::Response & res){
  
  Dynamixel.ledStatus(1,OFF);
  delay(20);
  Dynamixel.moveSpeed(14,offsetZero14,motorSpeed);
  delay(20);
  Dynamixel.moveSpeed(1,90*encTickToDeg+offsetZero1,motorSpeed);
  delay(20); 
  Dynamixel.ledStatus(1,ON);  
  return true;
}

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> server("release_arm",&release_arm);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> server2("grasp",&grasp);

void setup()
{ 
  Dynamixel.begin(1000000,2);  // Inicialize the servo at 1Mbps and Pin Control 2
  delay(20);
  Dynamixel.ledStatus(1,ON); 
  delay(20);
  Dynamixel.setTempLimit(1,80);  // Set Max Temperature to 80 Celcius
  delay(20);
  Dynamixel.setVoltageLimit(1,65,160);  // Set Operating Voltage from 6.5v to 16v
  delay(20);
  Dynamixel.setMaxTorque(1,512);        // 50% of Torque
  delay(20);
  Dynamixel.setSRL(1,2);                // Set the SRL to Return All
  delay(20);
  Dynamixel.ledStatus(14,ON); 
  delay(20);
  nh.initNode();
  nh.advertiseService(server);
  nh.advertiseService(server2);
  Dynamixel.move(1,90*encTickToDeg+offsetZero1);
  delay(20);
  Dynamixel.move(14,offsetZero14);
  delay(20);
  Dynamixel.ledStatus(1,OFF); 
  Dynamixel.ledStatus(14,OFF); 
}

void loop()
{  
  nh.spinOnce();
  delay(100);
}

