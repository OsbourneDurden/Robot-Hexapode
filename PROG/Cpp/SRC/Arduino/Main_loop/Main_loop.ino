#include <ros.h>
#include <std_msgs/Float32.h> // For the sonar data
//#include <std_msgs/Int16.h> // For the leg contacts data

const int sonarFrontPin = 8;
const int sonarBackPin = 9;

const int legsPins[6] = {2,3,4,5,6,7};

std_msgs::Float32 sonar_front_msg;
std_msgs::Float32 sonar_back_msg;
ros::Publisher pub_sonar_front("sonar_front", &sonar_front_msg);
ros::Publisher pub_sonar_back("sonar_back", &sonar_back_msg);

std_msgs::Float32 leg_msg_0;
std_msgs::Float32 leg_msg_1;
std_msgs::Float32 leg_msg_2;
std_msgs::Float32 leg_msg_3;
std_msgs::Float32 leg_msg_4;
std_msgs::Float32 leg_msg_5;

ros::Publisher pub_leg_0("leg_contact_0", &leg_msg_0);
ros::Publisher pub_leg_1("leg_contact_1", &leg_msg_1);
ros::Publisher pub_leg_2("leg_contact_2", &leg_msg_2);
ros::Publisher pub_leg_3("leg_contact_3", &leg_msg_3);
ros::Publisher pub_leg_4("leg_contact_4", &leg_msg_4);
ros::Publisher pub_leg_5("leg_contact_5", &leg_msg_5);

ros::NodeHandle nh;

long update_sonars_timeconstant_ms = 300;
long update_legs_timeconstant_ms = 20;
long update_IMU_timeconstant_ms = 300;
long update_IR_timeconstant_ms = 50;

long update_delay = 3;
boolean published = 0;

long last_publish_sonars;
long last_publish_legs;
long last_publish_IMU;
long last_publish_IR;

void setup()
{
  nh.initNode();
  nh.advertise(pub_sonar_front);
  nh.advertise(pub_sonar_back);
  
  for (int leg_number = 0; leg_number < 6; leg_number++) 
  {
    pinMode(legsPins[leg_number], INPUT);
  }
  
  nh.advertise(pub_leg_0);
  nh.advertise(pub_leg_1);
  nh.advertise(pub_leg_2);
  nh.advertise(pub_leg_3);
  nh.advertise(pub_leg_4);
  nh.advertise(pub_leg_5);
  
  last_publish_sonars = millis()  - update_sonars_timeconstant_ms;
  last_publish_legs = millis()  - update_legs_timeconstant_ms;
}

void loop()
{
  nh.spinOnce();
  published = 0;
  
  // Taking care of sonars
  if ((millis() - last_publish_sonars) > update_sonars_timeconstant_ms )
  {
    published = 1;
    // establish variables for duration of the ping,
    // and the distance result in inches and centimeters:
    long duration_front, duration_back, cm;
    
    pinMode(sonarFrontPin, OUTPUT);
    digitalWrite(sonarFrontPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sonarFrontPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sonarFrontPin, LOW);
    pinMode(sonarFrontPin, INPUT);
    duration_front = pulseIn(sonarFrontPin, HIGH);
    
    pinMode(sonarBackPin, OUTPUT);
    digitalWrite(sonarBackPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sonarBackPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sonarBackPin, LOW);
    pinMode(sonarBackPin, INPUT);
    duration_back = pulseIn(sonarBackPin, HIGH);
    
    // Publishing front
    cm = microsecondsToCentimeters(duration_front);
    sonar_front_msg.data = cm;
    pub_sonar_front.publish(&sonar_front_msg);
    //Now publishing back
    cm = microsecondsToCentimeters(duration_back);
    sonar_back_msg.data = cm;
    pub_sonar_back.publish(&sonar_back_msg);
    
    last_publish_sonars = millis(); 
  }
  
  // Taking care of legs contacts
  if ((millis() - last_publish_legs) > update_legs_timeconstant_ms )
  {
    published = 1;
   
    if (digitalRead(legsPins[0]) == HIGH) {leg_msg_0.data = 1;}
    else {leg_msg_0.data = 0;}
    pub_leg_0.publish(&leg_msg_0);
    
    if (digitalRead(legsPins[1]) == HIGH) {leg_msg_1.data = 1;}
    else {leg_msg_1.data = 0;}
    pub_leg_1.publish(&leg_msg_1);
    
    if (digitalRead(legsPins[2]) == HIGH) {leg_msg_2.data = 1;}
    else {leg_msg_2.data = 0;}
    pub_leg_2.publish(&leg_msg_2);
    
    if (digitalRead(legsPins[3]) == HIGH) {leg_msg_3.data = 1;}
    else {leg_msg_3.data = 0;}
    pub_leg_3.publish(&leg_msg_3);
    
    if (digitalRead(legsPins[4]) == HIGH) {leg_msg_4.data = 1;}
    else {leg_msg_4.data = 0;}
    pub_leg_4.publish(&leg_msg_4);
    
    if (digitalRead(legsPins[5]) == HIGH) {leg_msg_5.data = 1;}
    else {leg_msg_5.data = 0;}
    pub_leg_5.publish(&leg_msg_5);
    
    last_publish_legs=millis();
  }
  
  nh.spinOnce();
  if (published == 0) {delay(update_delay);}
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
