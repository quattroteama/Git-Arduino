#include <ros.h>
#include <std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include <Servo.h>

#define ENC_A 21
#define ENC_B 20
#define ENC_C 18
#define ENC_D 19

ros::NodeHandle  nh;

int wheel_1;
int wheel_2;
float mean_speed;
float x_vel_mps=0;

float rotation_left=0;
float rotation_right=0;

int angle;

int angle_1=0;
int angle_2=0;

float ang_vel;
float radius;

Servo servo1;
Servo servo2;

float R;

long previousMillis_left=0;
long previousMillis_right=0;

float time_right;
float time_left;

float velocity_left;
float velocity_right;

float ang_vel_left;
float ang_vel_right;

float linear_velocity;
float angular_velocity;

long pulse_count_a_b = 0;
boolean ENC_STATE_NEW_A, ENC_STATE_OLD_A = 0;
boolean ENC_STATE_NEW_B, ENC_STATE_OLD_B = 0;

long pulse_count_c_d = 0;
boolean ENC_STATE_NEW_C, ENC_STATE_OLD_C = 0;
boolean ENC_STATE_NEW_D, ENC_STATE_OLD_D = 0;



void velocity(const geometry_msgs::Twist& msg)
{
  x_vel_mps=msg.linear.x;
  ang_vel=msg.angular.z;
  
  mean_speed=85.2273*pow(abs(x_vel_mps),4)-27.1465*pow(abs(x_vel_mps),3)-226.9886*pow(abs(x_vel_mps),2)+396.5756*abs(x_vel_mps);

    R=1+(angle_2-angle_1)*0.12;
    wheel_2=(int)((2*mean_speed)/(1+1/R));
    wheel_1=(int)((wheel_2)/R);
    
    if(x_vel_mps>0)
    {
  
    digitalWrite(4,HIGH);
    digitalWrite(6,LOW);
    analogWrite(3,wheel_1);
    
    digitalWrite(8,HIGH);
    digitalWrite(10,LOW);
    analogWrite(11,wheel_2);
    }
    
    if(x_vel_mps<0)
    {
      digitalWrite(6,HIGH);
      digitalWrite(4,LOW);
      analogWrite(3,wheel_1);
    
      digitalWrite(10,HIGH);
      digitalWrite(8,LOW);
      analogWrite(11,wheel_2);
    }
    
    if(x_vel_mps==0)
    {
      digitalWrite(4,LOW);
      digitalWrite(6,LOW);
      digitalWrite(8,LOW);
      digitalWrite(10,LOW);
    }
    
    if(ang_vel==0)
  {
    servo1.write(96);
    servo2.write(88);
    angle_1=0;
    angle_2=0;
  }
  else
  {
    radius=(100*abs(x_vel_mps))/(ang_vel);
    if(radius>400 || radius<-400 || radius==0)
    {
      servo1.write(96);
      servo2.write(88);
      angle_1=0;
      angle_2=0;
    }
    else
    {
    angle=floor(((-6.31*pow(10,-6))*pow((abs(radius)),3))+(0.0038*pow((abs(radius)),2))+(-0.7151*(abs(radius)))+53.9288);
    if(radius<0)
     {
      angle_2=angle;
      angle_1=floor((angle_2/(0.97-0.01*angle_2))+angle_2*0.066);
      servo1.write(96-angle_1);
      servo2.write(88-angle_2);
     }
    else
    {
      angle_1=angle;
      angle_2=floor((angle_1/(0.97-0.01*angle_1))+angle_1*0.066);
      servo1.write(96+angle_1);
      servo2.write(88+angle_2);
    }
  }
  }
    
    
  
}

//void doSomething(const std_msgs::Float64& rot_speed)
//{
//    x_vel_mps=rot_speed.data;
//    
//    mean_speed=85.2273*pow(x_vel_mps,4)-27.1465*pow(x_vel_mps,3)-226.9886*pow(x_vel_mps,2)+396.5756*x_vel_mps;
//
//    R=1+(angle_2-angle_1)*0.12;
//    wheel_2=(int)((2*mean_speed)/(1+1/R));
//    wheel_1=(int)(wheel_2/R);
//  
//    digitalWrite(4,HIGH);
//    analogWrite(3,wheel_1);
//    
//    digitalWrite(8,HIGH);
//    analogWrite(11,wheel_2);
//
//
//}

//void doSomethingAgain(const std_msgs::Float64& servo_angle)
//{
//  ang_vel=servo_angle.data;
//  if(ang_vel==0)
//  {
//    servo1.write(81);
//    servo2.write(79);
//    angle_1=0;
//    angle_2=0;
//  }
//  else
//  {
//    radius=(100*x_vel_mps)/(ang_vel);
//    if(radius>400 || radius<-400 || radius==0)
//    {
//      servo1.write(81);
//      servo2.write(79);
//      angle_1=0;
//      angle_2=0;
//    }
//    else
//    {
//    angle=floor(((-6.31*pow(10,-6))*pow((abs(radius)),3))+(0.0038*pow((abs(radius)),2))+(-0.7151*(abs(radius)))+53.9288);
//    //angle=(atan(33/abs(radius)))*180/(2*3.14);
//    if(radius<0)
//     {
//      angle_2=angle;
//      angle_1=floor((angle_2/(0.97-0.01*angle_2))+angle_2*0.066);
//      servo1.write(81-angle_1);
//      servo2.write(79-angle_2);
//     }
//    else
//    {
//      angle_1=angle;
//      angle_2=floor((angle_1/(0.97-0.01*angle_1))+angle_1*0.066);
//      servo1.write(81+angle_1);
//      servo2.write(79+angle_2);
//    }
//  }
//  }
//  
//}


std_msgs::Float64 enc_a_b_data;
std_msgs::Float64 enc_c_d_data;

std_msgs::Float64 linear_vel_data;

std_msgs::Float64 ang_a_b_data;
std_msgs::Float64 ang_c_d_data;

std_msgs::Float64 angular_vel_data;

std_msgs::Float64 speed_pwm_data;

//geometry_msgs::Twist msg;

//ros::Publisher pub_enc_a_b( "left_linear_velocity", &enc_a_b_data);
//ros::Publisher pub_enc_c_d( "right_linear_velocity", &enc_c_d_data);
//
//ros::Publisher pub_ang_a_b( "left_angular_velocity", &ang_a_b_data);
//ros::Publisher pub_ang_c_d( "right_angular_velocity", &ang_c_d_data);
//
//ros::Publisher pub_pwm( "pwm", &speed_pwm_data);
//
//ros::Publisher pub_angular_vel( "angular_velocity", &angular_vel_data);
//ros::Publisher pub_linear_vel( "linear_velocity", &linear_vel_data);

//ros::Subscriber<std_msgs::Float64> rot("wheel_move", doSomething );
//ros::Subscriber<std_msgs::Float64> ang("wheel_turn", doSomethingAgain );

ros::Subscriber<geometry_msgs::Twist> vel("cmd_vel", velocity);


void setup()
{
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);
  pinMode(3,OUTPUT);
  digitalWrite(3,LOW);
  pinMode(6,OUTPUT);
  digitalWrite(6,LOW);
  
  pinMode(10,OUTPUT);
  digitalWrite(10,LOW);
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  pinMode(8,OUTPUT);
  digitalWrite(8,LOW);
  
  servo1.attach(13);
  servo2.attach(12);
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  //nh.subscribe(rot);
  //nh.subscribe(ang);
  nh.subscribe(vel);
//  nh.advertise(pub_enc_a_b);
//  nh.advertise(pub_enc_c_d);
//  nh.advertise(pub_ang_a_b);
//  nh.advertise(pub_ang_c_d);
//  nh.advertise(pub_pwm);
//  nh.advertise(pub_angular_vel);
//  nh.advertise(pub_linear_vel);  
  
  // Setup encoder pins
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH); // enable internal pull-up resistors
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH); // enable internal pull-up resistors
  pinMode(ENC_C, INPUT);
  digitalWrite(ENC_C, HIGH); // enable internal pull-up resistors
  pinMode(ENC_D, INPUT);
  digitalWrite(ENC_D, HIGH); // enable internal pull-up resistors
  
  

  attachInterrupt(2,isr_ENC_A,CHANGE);
  attachInterrupt(3,isr_ENC_B,CHANGE);
  attachInterrupt(5,isr_ENC_C,CHANGE);
  attachInterrupt(4,isr_ENC_D,CHANGE);

  ENC_STATE_OLD_A = digitalRead(ENC_A);
  ENC_STATE_OLD_B = digitalRead(ENC_B);
  ENC_STATE_OLD_C = digitalRead(ENC_C);
  ENC_STATE_OLD_D = digitalRead(ENC_D);

}

void isr_ENC_A(void)
{
  // increment pulse count
  ENC_STATE_NEW_A = digitalRead(ENC_A);
  if(ENC_STATE_NEW_A == ENC_STATE_OLD_A)
  {
      pulse_count_a_b++;

      
        
  }
}

void isr_ENC_B(void)
{
  // increment pulse count
  ENC_STATE_NEW_B = digitalRead(ENC_B);
  if(ENC_STATE_NEW_B == ENC_STATE_OLD_B)
  {
      pulse_count_a_b++;

      
        
  }
}

void isr_ENC_C(void)
{
  // increment pulse count
  ENC_STATE_NEW_C = digitalRead(ENC_C);
  if(ENC_STATE_NEW_C == ENC_STATE_OLD_C)
  {
      pulse_count_c_d++;

      
        
  }
}

void isr_ENC_D(void)
{
  // increment pulse count
  ENC_STATE_NEW_D = digitalRead(ENC_D);
  if(ENC_STATE_NEW_D == ENC_STATE_OLD_D)
  {
      pulse_count_c_d++;
      
        
  }
}


void loop()
{
  unsigned long currentMillis_left=millis();
  unsigned long currentMillis_right=millis();
  if(pulse_count_a_b>36)
   {
     rotation_left++;
     //enc_a_b_data.data=rotation_left;
     pulse_count_a_b=0;
     time_left=currentMillis_left-previousMillis_left;
     
     
     velocity_left=38/time_left;
     
     if(radius>0)
      {
        ang_vel_left=100*velocity_left/((radius-15));
      }
      else
      {
        ang_vel_left=-100*velocity_left/((abs(radius)+15));
      }
      
     
     enc_a_b_data.data=velocity_left;
     ang_a_b_data.data=ang_vel_left;
     previousMillis_left=currentMillis_left;
   }
   
   if(pulse_count_c_d>36)
    {
      rotation_right++;
      //enc_c_d_data.data=rotation_right;
      pulse_count_c_d=0;
      time_right=currentMillis_right-previousMillis_right;
      
      
      
      velocity_right=38/time_right;
      
      if(radius>0)
      {
        ang_vel_right=100*velocity_right/((radius+15));
      }
      else
      {
        ang_vel_right=-100*velocity_right/((abs(radius)-15));
      }
      
      
      enc_c_d_data.data=velocity_right;
      ang_c_d_data.data=ang_vel_right;
      previousMillis_right=currentMillis_right;
    }
    
    if(currentMillis_right-previousMillis_right>2000)
      {
        velocity_right=0;
        enc_c_d_data.data=velocity_right;
        ang_vel_right=0;
        ang_c_d_data.data=ang_vel_right;
      }
     
    if(currentMillis_left-previousMillis_left>2000)
     {
       velocity_left=0;
       enc_a_b_data.data=velocity_left;
       ang_vel_left=0;
       ang_a_b_data.data=ang_vel_left;
       
     }  
     
     if(abs(velocity_right-velocity_left)<0.05)
       {
         ang_vel_right=0;
         ang_c_d_data.data=ang_vel_right;
         ang_vel_left=0;
         ang_a_b_data.data=ang_vel_left;
       }
      
  linear_velocity=(velocity_left+velocity_right)/2;
  angular_velocity=(ang_vel_left+ang_vel_right)/2;
  
  linear_vel_data.data=linear_velocity;
  angular_vel_data.data=angular_velocity;
  
//  pub_angular_vel.publish(&angular_vel_data);
//  pub_linear_vel.publish(&linear_vel_data);
//  
//   
//  pub_enc_a_b.publish(&enc_a_b_data);
//  pub_enc_c_d.publish(&enc_c_d_data);
//  
//  pub_ang_a_b.publish(&ang_a_b_data);
//  pub_ang_c_d.publish(&ang_c_d_data);
  
  
  speed_pwm_data.data=mean_speed;
  //pub_pwm.publish(&speed_pwm_data);
  
  nh.spinOnce();
}
