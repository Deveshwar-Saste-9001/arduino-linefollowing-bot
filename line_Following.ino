
#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2

#define ena 5
#define m1a 6
#define m1b 7
#define m2a 9
#define m2b 8
#define enb 10

int LFSensor[5] =
{
  0, 0, 0, 0, 0
};

int iniMotorPowerL = 110;
int iniMotorPowerR = 97.5;

// PID controller
float Kp = -0.005;
float Ki = 0;// no need as there is already a ref point i.e the black line #steady error
float Kd = 26.77;
/*
  float Kp = 25;
  float Ki = 0.1;
  float Kd = 0.8;
*/
float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;


void stop()
{ // stop
  // motor 1
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, LOW);
  // motor 2
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, LOW);
}

void forward()
{ // forward
  // motor 1
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  // motor 2
  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, LOW);
}
void right()
{
  // right
  // motor 1
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  // motor 2
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, HIGH);
}
void left()
{
  // left
  // motor 1
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, HIGH);
  // motor 2
  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, LOW);
}

void setup()
{
  Serial.begin(115200);

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(ena, OUTPUT);

  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(enb, OUTPUT);

  // initial speed
  analogWrite(ena, 200);
  analogWrite(enb, 208);

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop()
{
  readLFSsensors();
  calculatePID();
  motorPIDcontrol();
  //delay(1);
}

// ---------------------------------------------------
void motorPIDcontrol()
{

  int leftMotorSpeed = constrain((iniMotorPowerL + PIDvalue), 0, 228);
  int rightMotorSpeed = constrain((iniMotorPowerR - PIDvalue), 0, 220);

  analogWrite(enb, leftMotorSpeed);
  analogWrite(ena, rightMotorSpeed);
  //forward();

  Serial.print(PIDvalue);
  Serial.print (" ==> Left, Right:  ");
  Serial.print (leftMotorSpeed);
  Serial.print ("   ");
  Serial.println (rightMotorSpeed);


}

// --------------------------------------------------------
void calculatePID()
{
  P = error;
  I = I + error;
  D = error - previousError;
  previousError = error;
  PIDvalue = ( (Kp * P) + (Ki * I) + (Kd * D));
  Serial.print(P);      //the first variable for plotting
  Serial.print(",");    //seperator
  Serial.print(I);
  Serial.print(",");
  Serial.println(D);

}


// -------------------------------------------------------------
/* read line sensors values
  Sensor Array  Error Value
  0 0 0 0 1  4
  0 0 0 1 1  3
  0 0 0 1 0  2
  0 0 1 1 0  1
  0 0 1 0 0  0
  0 1 1 0 0 -1
  0 1 0 0 0 -2
  1 1 0 0 0 -3
  1 0 0 0 0 -4
  1 1 1 1 1        0 Robot found continuous line : STOPPED
  0 0 0 0 0        0 Robot found no line: turn 180o
*/
void readLFSsensors()
{
  LFSensor[0] = digitalRead(A0);
  LFSensor[1] = digitalRead(A1);
  LFSensor[2] = digitalRead(A2);
  LFSensor[3] = digitalRead(A3);
  LFSensor[4] = digitalRead(A4);
//  digitalWrite(3,LFSensor[1]);
//  digitalWrite(5,LFSensor[2]);
//  digitalWrite(6,LFSensor[3]);
  for (int i = 0; i < 5; i++)
  {
    /* code */
    Serial.print(LFSensor[i]);
    //delay(200);
  }
  Serial.println(" ");
  if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 0))
  {
    //  mode = FOLLOWING_LINE;
    error = 5;
    right();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
  {
    //  mode = FOLLOWING_LINE;
    error = 4;
    right();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
  {
    //  mode = FOLLOWING_LINE;
    error = 3;
    right();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 0) && (LFSensor[4] == 1))
  {
    //  mode = FOLLOWING_LINE;
    error = 2;
    right();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 1))
  {
    //  mode = FOLLOWING_LINE;
    error = 1;
    right();
  }
 if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
  {
    //  mode = FOLLOWING_LINE;
    error = 0;
    forward();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
  {
    //  mode = FOLLOWING_LINE;
    error = -1;
    left();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 0) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
  {
    //  mode = FOLLOWING_LINE;
    error = -2;
    left();
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
  {
    //  mode = FOLLOWING_LINE;
    error = -3;
    left();
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
  {
    //  mode = FOLLOWING_LINE;
    error = -4;
    left();
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
  {
    //  mode = FOLLOWING_LINE;
    error = -5;
    left();
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
  {
    //  mode = STOPPED;
    error = 0;
    stop();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
  {
    ////  mode = NO_LINE;
    error = 0;
    forward();
    delay (50);
    analogWrite(ena, 0);
    analogWrite(enb, 0);
    stop();
  }

}