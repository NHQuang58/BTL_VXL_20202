/*define for L298 pins*/
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
#define ENA 3
#define ENB 11

/*define for IR sensor pins*/
#define S0  A5
#define S1  A4
#define S2  A3
#define S3  A2
#define S4  A1

float Kp = 40, Ki = 0, Kd = 160;
float error = 0, P = 0, I = 0, D = 0, pid_value = 0;
float pre_error = 0;
int sensorValue[5] = {0};
int init_value = 150;
int pid_right, pid_left = 0;

void read_sensor(void); //read value of sensors
void calc_pid(void); //calc parameters of PID
void drive_motor(void); //control DC motor


void setup() {
  
  pinMode(ENA, OUTPUT); //pwm pin 1
  pinMode(ENB, OUTPUT); //pwm pin 2
  pinMode(IN1, OUTPUT); //left 1
  pinMode(IN2, OUTPUT); //left 2
  pinMode(IN3, OUTPUT); //right 1
  pinMode(IN4, OUTPUT); //right 2  

  /*setup for Sensor IR pins*/
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

}

void loop() {

  read_sensor();
  calc_pid();
  drive_motor();

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
  D = error - pre_error;
  
  pid_value = (Kp * P) + (Ki * I) + (Kd * D);
  
  pre_error = error;
}

void drive_motor(void)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  pid_right = init_value - pid_value;
  pid_left  = init_value + pid_value;

  pid_right = constrain(pid_right, 0, 170);
  pid_left  = constrain(pid_left, 0, 170);  

  analogWrite(ENB, pid_left);
  analogWrite(ENA, pid_right);
  
}
