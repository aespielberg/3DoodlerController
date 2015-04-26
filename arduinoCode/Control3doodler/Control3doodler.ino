#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

int fastPinOut = 13;
int slowPinOut = 12;
int fastPinIn = 11;
int slowPinIn = 10;

bool off = false;
bool on = true;

char fast[5] = "fast";
char slow[5] = "slow";
char neither[4] = "off";

void fast_cb( const std_msgs::Bool& cmd_msg){
  if (cmd_msg.data == off)
  {
    digitalWrite(fastPinOut, LOW);  //pin is low, gate is closed, fast is on
  }
  else if (cmd_msg.data == on)
  {
    digitalWrite(slowPinOut, LOW);  //double check that slow is off
    digitalWrite(fastPinOut, HIGH);  //pin is high, gate is open, fast is on
  }
}

void slow_cb( const std_msgs::Bool& cmd_msg){
  if (cmd_msg.data == off)
  {
    digitalWrite(slowPinOut, LOW);  //pin is low, gate is closed, slow is on
  }
  else if (cmd_msg.data == on)
  {
    digitalWrite(fastPinOut, LOW);  //double check that fast is off
    digitalWrite(slowPinOut, HIGH);  //pin is high, gate is open, slow is on
  }
}

void status_cb( const std_msgs::Empty& empty_msg){
std_msgs::String str_msg;
ros::Publisher status("status", &str_msg);

  if (slowPinIn == HIGH)
  {
    str_msg.data = slow;
  }
  else if (fastPinIn == HIGH)
  {
    str_msg.data = fast;
  }    
  else 
  {
    str_msg.data = neither;
  }
  
  status.publish( &str_msg );
}

ros::Subscriber<std_msgs::Bool> fastSub("fast", fast_cb);
ros::Subscriber<std_msgs::Bool> slowSub("slow", slow_cb);
ros::Subscriber<std_msgs::Empty> statusSub("checkStatus", &status_cb );

void setup(){
  pinMode(fastPinOut, OUTPUT);
  pinMode(slowPinOut, OUTPUT);

  nh.initNode();
  nh.subscribe(fastSub);
  nh.subscribe(slowSub);
  nh.subscribe(statusSub);
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}
