#include <ros.h>
#include <std_msgs/Float32.h> // For the sonar data
#include <std_msgs/String.h> // For the leg contacts data

const int sonarFrontPin = 8;

const int legsPins[6] = {2,3,4,5,6,7};
char legs_string[12] = "0&0&0&0&0&0";

std_msgs::Float32 sonar_front_msg;
ros::Publisher pub_sonar_front("sonar_front", &sonar_front_msg);

std_msgs::String legs_msg;

ros::Publisher pub_legs("legs_contacts", &legs_msg);

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
  
  for (int leg_number = 0; leg_number < 6; leg_number++) 
  {
    pinMode(legsPins[leg_number], INPUT);
  }
  
  nh.advertise(pub_legs);
  
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
    
    // Publishing front
    cm = microsecondsToCentimeters(duration_front);
    sonar_front_msg.data = cm;
    pub_sonar_front.publish(&sonar_front_msg);
    
    last_publish_sonars = millis(); 
  }
  
  // Taking care of legs contacts
  if ((millis() - last_publish_legs) > update_legs_timeconstant_ms )
  {
    published = 1;
    
    if (digitalRead(legsPins[0]) == HIGH) {legs_string[0] = '1';}
    else {legs_string[0] = '0';}    
    if (digitalRead(legsPins[1]) == HIGH) {legs_string[2] = '1';}
    else {legs_string[2] = '0';}
    if (digitalRead(legsPins[2]) == HIGH) {legs_string[4] = '1';}
    else {legs_string[6] = '0';}
    if (digitalRead(legsPins[3]) == HIGH) {legs_string[6] = '1';}
    else {legs_string[6] = '0';}
    if (digitalRead(legsPins[4]) == HIGH) {legs_string[8] = '1';}
    else {legs_string[8] = '0';}
    if (digitalRead(legsPins[5]) == HIGH) {legs_string[10] = '1';}
    else {legs_string[10] = '0';}
    legs_msg.data = legs_string;
    pub_legs.publish(&legs_msg);
    
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
