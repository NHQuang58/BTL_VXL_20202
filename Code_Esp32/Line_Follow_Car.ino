/*define for L298 pins*/
#define IN1 22
#define IN2 21
#define IN3 19
#define IN4 18
#define ENA 23
#define ENB 5
/*define for sensor IR pins*/
#define S4  32
#define S3  33
#define S2  25
#define S1  26
#define S0  27
/*define for PWM*/
#define PWM_FREQUENCY   1000
#define PWM_RESOLUTION  8
#define PWM_CHANNEL_ENA 0
#define PWM_CHANNEL_ENB 1

/*global variables*/
float kP = 50, kI = 0, kD = 160;
float error = 0, P = 0, I = 0, D = 0, pidValue = 0, preError = 0;
int sensorValue[5] = {0}; //store value of sensors
int initSpeed = 150;
int pidLeft = 0, pidRight = 0;

void setup() {
  
  /*setup for L298 pins*/
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcSetup(PWM_CHANNEL_ENA, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_ENA);
  ledcSetup(PWM_CHANNEL_ENB, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ENB, PWM_CHANNEL_ENB);

  /*setup for Sensor IR pins*/
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*read_sensor();
  calc_pid();
  motor_control();*/
    ledcWrite(ENA, 100); //left motor speed
  ledcWrite(ENB, 100);  //right motor speed

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void read_sensor(void)
{
  sensorValue[0] = digitalRead(S0);  
  sensorValue[1] = digitalRead(S1);  
  sensorValue[2] = digitalRead(S2);  
  sensorValue[3] = digitalRead(S3);  
  sensorValue[4] = digitalRead(S4);  
  
  if((sensorValue[0]==0)&&(sensorValue[1]==0)&&(sensorValue[2]==0)&&(sensorValue[3]==0)&&(sensorValue[4]==1))
  error=4;
  else if((sensorValue[0]==0)&&(sensorValue[1]==0)&&(sensorValue[2]==0)&&(sensorValue[3]==1)&&(sensorValue[4]==1))
  error=3;
  else if((sensorValue[0]==0)&&(sensorValue[1]==0)&&(sensorValue[2]==0)&&(sensorValue[3]==1)&&(sensorValue[4]==0))
  error=2;
  else if((sensorValue[0]==0)&&(sensorValue[1]==0)&&(sensorValue[2]==1)&&(sensorValue[3]==1)&&(sensorValue[4]==0))
  error=1;
  else if((sensorValue[0]==0)&&(sensorValue[1]==0)&&(sensorValue[2]==1)&&(sensorValue[3]==0)&&(sensorValue[4]==0))
  error=0;
  else if((sensorValue[0]==0)&&(sensorValue[1]==1)&&(sensorValue[2]==1)&&(sensorValue[3]==0)&&(sensorValue[4]==0))
  error=-1;
  else if((sensorValue[0]==0)&&(sensorValue[1]==1)&&(sensorValue[2]==0)&&(sensorValue[3]==0)&&(sensorValue[4]==0))
  error=-2;
  else if((sensorValue[0]==1)&&(sensorValue[1]==1)&&(sensorValue[2]==0)&&(sensorValue[3]==0)&&(sensorValue[4]==0))
  error=-3;
  else if((sensorValue[0]==1)&&(sensorValue[1]==0)&&(sensorValue[2]==0)&&(sensorValue[3]==0)&&(sensorValue[4]==0))
  error=-4;
  else if((sensorValue[0]==0)&&(sensorValue[1]==0)&&(sensorValue[2]==0)&&(sensorValue[3]==0)&&(sensorValue[4]==0))
    if(error==-4) error=-5;
    else error=5;
}
void calc_pid(void)
{
  P = error;
  I = I + error;
  D = error - preError;
  
  pidValue = (kP * P) + (kI * I) + (kD * D);
  
  preError = error;
}
void motor_control(void)
{
  /*effective motor speed*/
  pidLeft = initSpeed - pidValue;
  pidRight = initSpeed + pidValue;

  /*pulse limit*/
  constrain(pidLeft, 0 , 255);
  constrain(pidRight, 0 , 255);

  ledcWrite(ENA, pidRight); //left motor speed
  ledcWrite(ENB, pidLeft);  //right motor speed

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
