#include<TimerOne.h>

double tocdo,tocdodat,error,xung;
double t;
double proportional, integral, previous1, previous2, output = 0;
double kp, ki, kd;
#define PWM 5
#define DIR1 6  
#define DIR2 7

void setup()
{
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(PWM,OUTPUT);
  pinMode(DIR1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000001;
  tocdodat = 30;
  tocdo = 0;
  t = 0.01;
  kp = 3;
  ki = 0.1;
  kd = 0;
  Serial.begin(9600);
  attachInterrupt(0,Demxung,FALLING);
  Timer1.initialize(10000);
  Timer1.attachInterrupt(pid_roirac);
  
}

void pid_roirac()
{
  tocdo = ((xung/234)*(1/t)*60);
  xung=0;
  error = tocdodat - tocdo;
  double proportion = kp * (error -previous1);
  double integral = ki * (error + previous1) * t/2;
  double derivative = kd * (error - 2* previous1 + previous2) / t;
  previous2 = previous1;
  previous1 = error;
  output += proportional + integral + derivative;

  if(output > 255)
  output = 255;
  if(output < -255)
  output = -255
  if( output >= 0)
  {
  analogWrite(PWM,output);
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,LOW);
  }
 else
  {
  analogWrite(PWM,output);
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,HIGH);
  }
}

void loop(){
    delay(100);
    Serial.println(tocdo);
}

void Demxung(){
  if(digitalRead(3) == LOW)
  xung++; 
  else 
  xung--;
}