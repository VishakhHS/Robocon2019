

/*
      pwm3
  pwm2    pwm1
      pwm4
*/
#include <Servo.h>

Servo myservo;

#define Motor_Left_PWM 10
#define Motor_Right_PWM 4
#define Motor_Front_PWM 12
#define Motor_Back_PWM 6

#define Motor_Left_Dir 9
#define Motor_Right_Dir 3
#define Motor_Front_Dir 11
#define Motor_Back_Dir 5

/****** Directions ******/
#define Forward 0
#define Backward 1
#define Left 2
#define Right 3
#define Wall_Detected 4

/**** Pin Configuration Variables ****/
unsigned int V_S_0_Pin = 0;
unsigned int V_S_1_Pin = 1;

unsigned int V_F_0_Pin = 2;
unsigned int V_F_1_Pin = 3;

/****** Values of ultasonic sensors ****/
unsigned int V_F_0 = 0;
unsigned int V_F_1 = 0;

unsigned int V_S_0 = 0;
unsigned int V_S_1 = 0;

/***** Average Values of Front and Side Sensors ******/
float Avg_F = 0;
float Avg_S = 0;

/*******Base speed for both motors**********/
int BaseSpeed = 150;

/*********Errors and Corrections******/
int Error = 0;
int Prev_Error = 0;
int Corr = 0;
int Corr_Int;

/*******PID Constants*********/
const float Kp = 0.28; //0.3
const float Kd = 0.15; //0.15
float Ki = 0.3; //0.3

/******* Setpoints for front and side ******/
int I_S = 600;
int I_F = 530;

/******* Analog Output Values *********/
unsigned int OCR_Left = 0;
unsigned int OCR_Right = 0;
unsigned int OCR_Front = 0;
unsigned int OCR_Back = 0;

#define      TRUE  1U
#define      FALSE 0U

const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
const byte rx1 = 0;
const byte rx2 = 0;
const byte jPulse = A8;   // Connect JPULSE of LSA08 to pin 4
const byte jPulse2 = 14;
const byte jPulse3 = 53;
const byte jPulse4 = 37;
const byte digital_1 = 21;
const byte digital_2 = A15;
//const byte digital_3 = 39;
const byte dir1 = 3;
const byte dir2 = 9;
const byte dir3 = 11;
const byte dir4 = 5;

const byte pwm1 = 4;
const byte pwm2 = 10;
const byte pwm3 = 12;
const byte pwm4 = 6;

const byte laser1 = A4;
const byte laser2 = A7;
const byte laser3 = A6;
const byte laser4 = A5;
//const byte button = 9;
int laser1x, laser2x, laser3x, laser4x;

//const byte actuate = 9;
const byte button1x = 42;
const byte button2x = 44;
const byte button1y = 46;
const byte button2y = 48;
const byte button1z = 50;
const byte button2z = 52;

int buttonState1x = 0;
int buttonState2x = 0;
int buttonState1y = 0;
int buttonState2y = 0;
int buttonState1z = 0;
int buttonState2z = 0;

int case3_test_case = 0;

int actuateState = 0;
//////////////////////////TEST FLAGS////////////////////////

bool case_three_execution_status = FALSE;

//////////////////////////////////////////////////////////

unsigned int junctionCount = 0;   // Variable to store junction count value
unsigned int junctionCount2 = 0;
unsigned int junctionCount3 = 0;
unsigned int junctionCount4 = 0;

unsigned int digital_stop_1 = 0;
unsigned int digital_stop_2 = 0;
unsigned int digital_stop_3 = 0;
int flag1 = 0;
int flag1x = 0;
int flag1y = 0;
int flag1z = 0;
int flag1n = 0;
int flagtz = 0;
int flag1s = 0;
int flag5 = 0;
int flag1v = 0;
int flagstart = 0;
int flagstart2 = 0;
int flagtzstop = 0;
unsigned int laser_interrupt_1;
unsigned int laser_interrupt_2;
unsigned int laser_interrupt_3;
unsigned int laser_interrupt_4;
int a = 0;
int b = 0;
int c = 0;
int m = 0, n = 0;
int t = 0, up = 0;

int u, v, w, x, y, z;
int u1, v1, w1, x1, y1, z1;
int u2, v2, w2, x2, y2, z2;
int u3, v3, w3, x3, y3, z3;
int m3 = 0, n3 = 0;
int mx = 0, nx = 0;
int  flag = 0;
int flagtz1 = 0;
int flagtz1x = 0;
int flagtz1y = 0;
int flag5n = 0;
int flagsh = 0;
int flagshuruwat = 0;
int flagsuruwat = 0;
int flagmanual = 0;
int flagcase1 = 0;
int flagcase2 = 0;
int flagcase3 = 0;
int m4 = 0, n4 = 0;
int ms = 0, ns = 0;
int ma = 0, na = 0;
int mq = 0, nq = 0;
int mw = 0, nw = 0;


volatile char err = 0, lerr = 0;
volatile float corr = 0;
volatile float kp = 0.4, pro = 0;//0.5
volatile float ki = 0.0, intg = 0, I = 0;//0.00001
volatile float kd = 0.0, diff = 0;//0.0002
volatile float pwm = 0;


/****** PID Tuning for side sensors ******
  Inputs: V_0 = V_S_0 / V_F_0
        : V_1 = V_S_1 / V_F_1
        : Direction : 1) Forward
                      2) Backward
                     3) Left
                      4) Right
*/
void CalculatePIDCorrectionSide(int V_0 , int V_1 , char Direction)
{
  Error = abs(V_0 - V_1);

  if (Error > 40)
    Error = 40;
  else if (Error < -40)
    Error = -40;
  else if (Error < 2 && Error > -2)
    Error = 0;

  if (Prev_Error < 0)
    Prev_Error = 0;
  else if (Prev_Error - Error <= 0)
    Prev_Error = Error;

  if (Direction == Forward || Direction == Backward)
  {
    Corr = Kp * Error + Kd * (Prev_Error - Error);
    Corr_Int = Ki * (I_S - Avg_S);
  }
  else if (Direction == Left || Direction == Right)
  {
    Corr = Kp * Error + Kd * (Prev_Error - Error);
    Corr_Int = Ki * (I_F - Avg_F);
  }

  if (Corr_Int > 55)
    Corr_Int = 55;
  else if (Corr_Int < -55)
    Corr_Int = -55;
}

/********** Apply Changes to Side Motors *********/
void VelocityForwardMotors(int Direction)
{
  //OCR_Front=0;
  //OCR_Back=0;

  if (Direction == Forward)
  {
    if (V_S_0 < V_S_1)
    {
      OCR_Left = BaseSpeed - Corr - Corr_Int;
      OCR_Right = BaseSpeed + Corr + Corr_Int;
    }

    else if (V_S_0 > V_S_1)
    {
      OCR_Left = BaseSpeed + Corr - Corr_Int;
      OCR_Right = BaseSpeed - Corr + Corr_Int;
    }

    else if (V_S_0 == V_S_1)
    {
      OCR_Left = BaseSpeed - Corr_Int;
      OCR_Right = BaseSpeed + Corr_Int;
    }
  }
  else if (Direction == Backward)
  {
    if (V_S_1 < V_S_0)
    {
      OCR_Left = BaseSpeed - Corr - Corr_Int;
      OCR_Right = BaseSpeed + Corr + Corr_Int;
    }

    else if (V_S_1 > V_S_0)
    {
      OCR_Left = BaseSpeed + Corr - Corr_Int;
      OCR_Right = BaseSpeed - Corr + Corr_Int;
    }

    else if (V_S_0 == V_S_1)
    {
      OCR_Left = BaseSpeed - Corr_Int;
      OCR_Right = BaseSpeed + Corr_Int;
    }
  }
  Prev_Error = Error;
}

/******** Apply Changes to Front motors ********/
void VelocityLeftMotors(int Direction)
{
  //OCR_Left=0;
  //OCR_Right=0;

  if (Direction == Left)
  {
    if (V_F_0 < V_F_1)
    {
      OCR_Back = BaseSpeed - Corr - Corr_Int;
      OCR_Front = BaseSpeed + Corr + Corr_Int;
    }

    else if (V_F_0 > V_F_1)
    {
      OCR_Back = BaseSpeed + Corr - Corr_Int;
      OCR_Front = BaseSpeed - Corr + Corr_Int;
    }

    else if (V_F_0 == V_F_1)
    {
      OCR_Back = BaseSpeed - Corr_Int;
      OCR_Front = BaseSpeed + Corr_Int;
    }
  }
  else if (Direction == Right)
  {
    if (V_F_1 < V_F_0)
    {
      OCR_Back = BaseSpeed - Corr - Corr_Int;
      OCR_Front = BaseSpeed + Corr + Corr_Int;
    }

    else if (V_F_1 > V_F_0)
    {
      OCR_Back = BaseSpeed + Corr - Corr_Int;
      OCR_Front = BaseSpeed - Corr + Corr_Int;
    }

    else if (V_F_0 == V_F_1)
    {
      OCR_Back = BaseSpeed - Corr_Int;
      OCR_Front = BaseSpeed + Corr_Int;
    }
  }
  Prev_Error = Error;
}

/**** Move Forward/Backward *******/
/*
   Input:
   Direction = Forward/Backward
*/

void Move_Forward_Backward(int Direction)
{
  /***** Check if front Wall is reached ******/
  /***** Change speed accordingly *****/
  //CheckForObstacle();

  if (Direction == Forward)
    CalculatePIDCorrectionSide(V_S_0, V_S_1, Forward);
  if (Direction == Backward)
    CalculatePIDCorrectionSide(V_S_1, V_S_0, Backward);

  ApplyChangesToHardware(Direction);

  /****** Change Velocity to follow line forward ******/
  VelocityForwardMotors(Direction);
}

/***** Move Left or Right ****/
/*
   Input:
   Direction = Left/Right
*/

void Move_Left_Right(int Direction)
{
  /***** Calculate corrections applying PID ****/
  if (Direction == Left)
    CalculatePIDCorrectionSide(V_F_0, V_F_1, Left);
  if (Direction == Right)
    CalculatePIDCorrectionSide(V_F_1, V_F_0, Right);

  ApplyChangesToHardware(Direction);
  /****** Change Velocity to follow line****/
  VelocityLeftMotors(Direction);
}

void Wall_Follow(int Direction)
{
  if (Direction == Forward || Direction == Backward)
    Move_Forward_Backward(Direction);
  else if (Direction == Left || Direction == Right)
    Move_Left_Right(Direction);
}

void ReadUltrasonicSensorValues(void)
{
  V_F_0 = analogRead(V_F_0_Pin);
  V_F_1 = analogRead(V_F_1_Pin);

  V_S_0 = analogRead(V_S_0_Pin);
  V_S_1 = analogRead(V_S_1_Pin);
}

void ApplyChangesToHardware(int Direction)
{
  if (Direction == Forward || Direction == Backward)
  {
    analogWrite(Motor_Left_PWM, OCR_Left);
    analogWrite(Motor_Right_PWM, OCR_Right);
  }
  else if (Direction == Left || Direction == Right)
  {
    analogWrite(Motor_Front_PWM, OCR_Front);
    analogWrite(Motor_Back_PWM, OCR_Back);
  }

  if (Direction == Left)
  {
    digitalWrite(Motor_Front_Dir, LOW);
    digitalWrite(Motor_Back_Dir, LOW);
  }

  else if (Direction == Right)
  {
    digitalWrite(Motor_Front_Dir, HIGH);
    digitalWrite(Motor_Back_Dir, HIGH);
  }

  if (Direction == Forward)
  {
    digitalWrite(Motor_Left_Dir, LOW);
    digitalWrite(Motor_Right_Dir, HIGH);
  }

  else if (Direction == Backward)
  {
    digitalWrite(Motor_Left_Dir, HIGH);
    digitalWrite(Motor_Right_Dir, LOW);
  }
}

//volatile int err = 0;

int laser;
void setup() {
  myservo.attach(13);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(dir4, OUTPUT);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);

  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);

  pinMode(23, INPUT);
  pinMode(25, INPUT);
  pinMode(27, INPUT);
  pinMode(29, INPUT);
  pinMode(31, INPUT);
  pinMode(33, INPUT);

  pinMode(41, INPUT);
  pinMode(43, INPUT);
  pinMode(45, INPUT);
  pinMode(47, INPUT);
  pinMode(49, INPUT);
  pinMode(51, INPUT);

  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);
  pinMode(A14, INPUT);

  pinMode(jPulse, INPUT);
  pinMode(jPulse2, INPUT);
  pinMode(jPulse3, INPUT);
  pinMode(jPulse4, INPUT);

  pinMode(button1x, OUTPUT);
  pinMode(button2y, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(52, OUTPUT);

  //pinMode(rx, INPUT);

  //
  //  pinMode(button1, OUTPUT);
  //  pinMode(button2, OUTPUT);
  //pinMode(13, OUTPUT);
  //pinMode(12, OUTPUT);
  // pinMode(dir, OUTPUT);
  // pinMode(pwm8, OUTPUT);
  //pinMode(actuate, OUTPUT);


  pinMode(digital_1, INPUT);
  pinMode(digital_2, INPUT);


  pinMode(laser1, INPUT);
  pinMode(laser2, INPUT);
  pinMode(laser3, INPUT);
  pinMode(laser4, INPUT);


  // digitalWrite(4,HIGH);
  Serial.begin(9600);
  //Serial1.begin(115200);
  //Serial2.begin(9600);
  //Serial3.begin(9600);
  myservo.write(0);
  while (flagsuruwat == 0)
  {

    /* while (1)
      {
      // BaseSpeed = 140;
           ///////////Read Values from front and side sensors ///////////
           ReadUltrasonicSensorValues();

           ///////// Calculate Average Values /////////////////
        //   Avg_S = (V_S_0 + V_S_1) / 2.0;
        //   Avg_F = (V_F_0 + V_F_1) / 2.0;
           Serial.println(V_F_0);
           Serial.println(V_F_1);
           Wall_Follow(Left);
    */
    /*  u2 = digitalRead(15);
      v2 = digitalRead(16);
      w2 = digitalRead(17);
      x2 = digitalRead(18);
      y2 = digitalRead(19);
      z2 = digitalRead(20);
      line2();
      ///////////////////////////////////////////////////
      if ((u2 == 1) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 0))
      {
        analogWrite(pwm2, 100);//28
        digitalWrite(dir2, HIGH);
      }

      if ((u2 == 1) && (v2 == 1) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 0))
      {
        analogWrite(pwm2, 60);//22
        digitalWrite(dir2, HIGH);
      }

      if ((u2 == 0) && (v2 == 1) && (w2 == 1) && (x2 == 0) && (y2 == 0) && (z2 == 0))
      {
        analogWrite(pwm2, 30);
        digitalWrite(dir2, HIGH);
      }

      if ((u2 == 0) && (v2 == 0) && (w2 == 1) && (x2 == 1) && (y2 == 0) && (z2 == 0))
      {
        analogWrite(pwm2, 0);
        digitalWrite(dir2, LOW);
      }

      if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 1) && (y2 == 1) && (z2 == 0))
      {
        analogWrite(pwm2, 30);
        digitalWrite(dir2, LOW);
      }

      if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 1) && (z2 == 1))
      {
        analogWrite(pwm2, 60);
        digitalWrite(dir2, LOW);
      }

      if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 1))
      {
        analogWrite(pwm2, 100);
        digitalWrite(dir2, LOW);
      }
      /////////////////////////////////////////////////////
      analogWrite(pwm4, 100);//65  //150
      analogWrite(pwm3, 105);//70  //155
      // line2();
      digitalWrite(dir3, LOW);
      digitalWrite(dir4, LOW);


      } */
    digitalWrite(dir3, LOW);
    digitalWrite(dir4, LOW);
    analogWrite(pwm3, 40);
    analogWrite(pwm4, 40);
    Serial.println("cover");
    digital_stop_2 = digitalRead(digital_2);
    if ((digital_stop_2 == 1) && (flagshuruwat == 0))
    {

      flagshuruwat = 1;


      while (flagshuruwat == 1)
      {
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);

        //////**


        u1 = digitalRead(A9);
        v1 = digitalRead(A10);
        w1 = digitalRead(A11);
        x1 = digitalRead(A12);
        y1 = digitalRead(A13);
        z1 = digitalRead(A14);

        Serial.println("start");
        BaseSpeed = 100;
        /****** Read Values from front and side sensors *****/
        ReadUltrasonicSensorValues();

        /***** Calculate Average Values *******/
        Avg_S = (V_S_0 + V_S_1) / 2.0;
        Avg_F = (V_F_0 + V_F_1) / 2.0;

        Wall_Follow(Forward);

        analogWrite(pwm4, 0);
        //a = Serial1.read();
        //1 Serial.print("a == ");
        //Serial.println(a);
        //Serial1.begin(115200);
        digital_stop_1 = digitalRead(digital_1);
        line1();
        // line_follow();
        //-------------------------------------------
        junctionCount = digitalRead(jPulse);
        if ((junctionCount == 1) && (flag1 == 0))
        {


          u1 = digitalRead(A9);
          v1 = digitalRead(A10);
          w1 = digitalRead(A11);
          x1 = digitalRead(A12);
          y1 = digitalRead(A13);
          z1 = digitalRead(A14);

          flag1 = 1;
          line1();
          analogWrite(pwm4, 0);

          while (flag1 == 1)
          {
            digital_stop_1 = digitalRead(digital_1);

            u1 = digitalRead(A9);
            v1 = digitalRead(A10);
            w1 = digitalRead(A11);
            x1 = digitalRead(A12);
            y1 = digitalRead(A13);
            z1 = digitalRead(A14);

            BaseSpeed = 100;
            /****** Read Values from front and side sensors *****/
            ReadUltrasonicSensorValues();

            /***** Calculate Average Values *******/
            Avg_S = (V_S_0 + V_S_1) / 2.0;
            Avg_F = (V_F_0 + V_F_1) / 2.0;

            Wall_Follow(Forward);


            line1();
            if (digital_stop_1 == 1)
            {
              line1();
              flag1 = 2;
              while (flag1 == 2)
              {
                line1();

                u1 = digitalRead(A9);
                v1 = digitalRead(A10);
                w1 = digitalRead(A11);
                x1 = digitalRead(A12);
                y1 = digitalRead(A13);
                z1 = digitalRead(A14);
                Serial.println("Digital!!!!!!!!!!!!!!!!!!!!");

                BaseSpeed = 100;
                /****** Read Values from front and side sensors *****/
                ReadUltrasonicSensorValues();

                /***** Calculate Average Values *******/
                Avg_S = (V_S_0 + V_S_1) / 2.0;
                Avg_F = (V_F_0 + V_F_1) / 2.0;

                Wall_Follow(Forward);


                junctionCount = digitalRead(jPulse);
                if (junctionCount == 1)
                {
                  u1 = digitalRead(A9);
                  v1 = digitalRead(A10);
                  w1 = digitalRead(A11);
                  x1 = digitalRead(A12);
                  y1 = digitalRead(A13);
                  z1 = digitalRead(A14);

                  line1();
                  flag1 = 3;

                  Serial.println("start_junction");
                  // flag1 = 3;
                  while (flag1 == 3)
                  {
                    //Serial1.begin(115200);
                    digital_stop_1 = digitalRead(digital_1);
                    analogWrite(pwm4, 0);

                    //  BaseSpeed = 50;
                    /****** Read Values from front and side sensors *****/
                    // ReadUltrasonicSensorValues();

                    /***** Calculate Average Values *******/
                    // Avg_S = (V_S_0 + V_S_1) / 2.0;
                    // Avg_F = (V_F_0 + V_F_1) / 2.0;

                    // Wall_Follow(Forward);
                    analogWrite(pwm1, 40);
                    analogWrite(pwm2, 40);
                    digitalWrite(dir1, HIGH);
                    digitalWrite(dir2, LOW);

                    u1 = digitalRead(A9);
                    v1 = digitalRead(A10);
                    w1 = digitalRead(A11);
                    x1 = digitalRead(A12);
                    y1 = digitalRead(A13);
                    z1 = digitalRead(A14);
                    line1();
                    //line_follow();
                    flagstart = 1;
                    if (digital_stop_1 == 1)
                    {
                      Serial.println("start_stop2222");
                      digital_stop_1 = digitalRead(digital_1);
                      flagstart = 2;

                      while (flagstart == 2)
                      {
                        Serial.println("start_stop");
                        OCR_Left = 0;
                        OCR_Right = 0;
                        analogWrite(pwm1, 0);
                        analogWrite(pwm2, 0);
                        analogWrite(pwm3, 0);
                        analogWrite(pwm4, 0);
                        flagstart2 = 1;
                        laser_interrupt_1 = digitalRead(laser1);
                        laser_interrupt_2 = digitalRead(laser2);
                        laser_interrupt_3 = digitalRead(laser3);
                        flagshuruwat = 9;
                        flagsuruwat = 9;
                        flag1 = 5;
                        //   flag1 = 9;
                        flagstart = 3;
                        m = 0;
                        n = 0;
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }

    //-------------------------------------------
  }
  ////////////*



}

void line()
{
  if ((u == 1) && (v == 0) && (w == 0) && (x == 0) && (y == 0) && (z == 0))
  {
    analogWrite(pwm1, 150);//22
    digitalWrite(dir1, LOW);
  }

  if ((u == 1) && (v == 1) && (w == 0) && (x == 0) && (y == 0) && (z == 0))
  {
    analogWrite(pwm1, 100);//18
    digitalWrite(dir1, LOW);
  }
  if ((u == 0) && (v == 1) && (w == 1) && (x == 0) && (y == 0) && (z == 0))
  {
    analogWrite(pwm1, 50);//18
    digitalWrite(dir1, LOW);
  }

  if ((u == 0) && (v == 0) && (w == 1) && (x == 1) && (y == 0) && (z == 0))
  {
    analogWrite(pwm1, 0);
    digitalWrite(dir1, LOW);
  }

  if ((u == 0) && (v == 0) && (w == 0) && (x == 1) && (y == 1) && (z == 0))
  {
    analogWrite(pwm1, 50);
    digitalWrite(dir1, HIGH);
  }

  if ((u == 0) && (v == 0) && (w == 0) && (x == 0) && (y == 1) && (z == 1))
  {
    analogWrite(pwm1, 100);
    digitalWrite(dir1, HIGH);
  }

  if ((u == 0) && (v == 0) && (w == 0) && (x == 0) && (y == 0) && (z == 1))
  {
    analogWrite(pwm1, 150);
    digitalWrite(dir1, HIGH);
  }


  //  if ((w == 1) && (x == 1) && (y == 1) && (z == 1))
  //  {
  //    //    analogWrite(pwm1, 40);
  //    //    analogWrite(pwm2, 40);
  //    //
  //    //    digitalWrite(dir1, HIGH);
  //    //    digitalWrite(dir2, HIGH);
  //  }

}

void line1()
{
  if ((u1 == 1) && (v1 == 0) && (w1 == 0) && (x1 == 0) && (y1 == 0) && (z1 == 0))
  {
    analogWrite(pwm3, 100);
    digitalWrite(dir3, LOW);
  }
  if ((u1 == 1) && (v1 == 1) && (w1 == 0) && (x1 == 0) && (y1 == 0) && (z1 == 0))
  {
    analogWrite(pwm3, 60);
    digitalWrite(dir3, LOW);
  }

  if ((u1 == 0) && (v1 == 1) && (w1 == 1) && (x1 == 0) && (y1 == 0) && (z1 == 0))
  {
    analogWrite(pwm3, 30);
    digitalWrite(dir3, LOW);
  }

  if ((u1 == 0) && (v1 == 0) && (w1 == 1) && (x1 == 1) && (y1 == 0) && (z1 == 0))
  {
    analogWrite(pwm3, 0);
    digitalWrite(dir3, HIGH);
  }

  if ((u1 == 0) && (v1 == 0) && (w1 == 0) && (x1 == 1) && (y1 == 1) && (z1 == 0))
  {
    analogWrite(pwm3, 30);
    digitalWrite(dir3, HIGH);
  }

  if ((u1 == 0) && (v1 == 0) && (w1 == 0) && (x1 == 0) && (y1 == 1) && (z1 == 1))
  {
    analogWrite(pwm3, 60);
    digitalWrite(dir3, HIGH);
  }
  if ((u1 == 0) && (v1 == 0) && (w1 == 0) && (x1 == 0) && (y1 == 0) && (z1 == 1))
  {
    analogWrite(pwm3, 100);
    digitalWrite(dir3, HIGH);
  }

  //  if ((w == 1) && (x == 1) && (y == 1) && (z == 1))
  //  {
  //    //    analogWrite(pwm1, 40);
  //    //    analogWrite(pwm2, 40);
  //    //
  //    //    digitalWrite(dir1, HIGH);
  //    //    digitalWrite(dir2, HIGH);
  //  }

}

void line2()
{
  if ((u2 == 1) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 0))
  {
    analogWrite(pwm2, 100);//28
    digitalWrite(dir2, HIGH);
  }

  if ((u2 == 1) && (v2 == 1) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 0))
  {
    analogWrite(pwm2, 60);//22
    digitalWrite(dir2, HIGH);
  }

  if ((u2 == 0) && (v2 == 1) && (w2 == 1) && (x2 == 0) && (y2 == 0) && (z2 == 0))
  {
    analogWrite(pwm2, 30);
    digitalWrite(dir2, HIGH);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 1) && (x2 == 1) && (y2 == 0) && (z2 == 0))
  {
    analogWrite(pwm2, 0);
    digitalWrite(dir2, LOW);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 1) && (y2 == 1) && (z2 == 0))
  {
    analogWrite(pwm2, 30);
    digitalWrite(dir2, LOW);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 1) && (z2 == 1))
  {
    analogWrite(pwm2, 60);
    digitalWrite(dir2, LOW);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 1))
  {
    analogWrite(pwm2, 100);
    digitalWrite(dir2, LOW);
  }

  //  if ((w == 1) && (x == 1) && (y == 1) && (z == 1))
  //  {
  //    //    analogWrite(pwm1, 40);
  //    //    analogWrite(pwm2, 40);
  //    //
  //    //    digitalWrite(dir1, HIGH);
  //    //    digitalWrite(dir2, HIGH);
  //  }

}


void line_follow()
{
  if ((u3 == 1) && (v3 == 0) && (w3 == 0) && (x3 == 0) && (y3 == 0) && (z3 == 0))
  {
    analogWrite(pwm4, 200);//28
    digitalWrite(dir4, HIGH);
  }

  if ((u3 == 1) && (v3 == 1) && (w3 == 0) && (x3 == 0) && (y3 == 0) && (z3 == 0))
  {
    analogWrite(pwm4, 150);//22
    digitalWrite(dir4, HIGH);
  }

  if ((u3 == 0) && (v3 == 1) && (w3 == 1) && (x3 == 0) && (y3 == 0) && (z3 == 0))
  {
    analogWrite(pwm4, 40);
    digitalWrite(dir4, HIGH);
  }

  if ((u3 == 0) && (v3 == 0) && (w3 == 1) && (x3 == 1) && (y3 == 0) && (z3 == 0))
  {
    analogWrite(pwm4, 0);
    digitalWrite(dir4, LOW);
  }

  if ((u3 == 0) && (v3 == 0) && (w3 == 0) && (x3 == 1) && (y3 == 1) && (z3 == 0))
  {
    analogWrite(pwm4, 40);
    digitalWrite(dir4, LOW);
  }

  if ((u3 == 0) && (v3 == 0) && (w3 == 0) && (x3 == 0) && (y3 == 1) && (z3 == 1))
  {
    analogWrite(pwm4, 150);
    digitalWrite(dir4, LOW);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 1))
  {
    analogWrite(pwm4, 200);
    digitalWrite(dir4, LOW);
  }

}

void loop()
{

  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0);
  analogWrite(pwm4, 0);

  //a = Serial1.read();
  Serial.println("baher gelay111111111111111111111111111111111111111111111111");
  //  b = Serial2.read();
  //c = Serial3.read();

  u = digitalRead(41);
  v = digitalRead(43);
  w = digitalRead(45);
  x = digitalRead(47);
  y = digitalRead(49);
  z = digitalRead(51);

  u1 = digitalRead(A9);
  v1 = digitalRead(A10);
  w1 = digitalRead(A11);
  x1 = digitalRead(A12);
  y1 = digitalRead(A13);
  z1 = digitalRead(A14);


  u2 = digitalRead(15);
  v2 = digitalRead(16);
  w2 = digitalRead(17);
  x2 = digitalRead(18);
  y2 = digitalRead(19);
  z2 = digitalRead(20);

  u3 = digitalRead(35);
  v3 = digitalRead(33);
  w3 = digitalRead(31);
  x3 = digitalRead(29);
  y3 = digitalRead(27);
  z3 = digitalRead(25);
  //  j = digitalRead(A8);
  // d = digitalRead(A15);

  junctionCount = digitalRead(jPulse);
  junctionCount2 = digitalRead(jPulse2);
  digital_stop_1 = digitalRead(digital_1);
  digital_stop_2 = digitalRead(digital_2);
  laser_interrupt_1 = digitalRead(laser1);
  laser_interrupt_2 = digitalRead(laser2);
  laser_interrupt_3 = digitalRead(laser3);
  laser_interrupt_4 = digitalRead(laser4);

  buttonState1x = digitalRead(button1x);
  buttonState2x = digitalRead(button2x);
  buttonState1y = digitalRead(button1y);
  buttonState2y = digitalRead(button2y);
  buttonState1z = digitalRead(button1z);
  buttonState2z = digitalRead(button2z);
  //  actuateState = digitalRead(actuate);


  if (laser_interrupt_1 == 1) 
  {
    
    laser = 1;
    flagcase1 = 1;
  }



  switch (laser)
  {


    case 0:
      analogWrite(pwm1, 0);
      analogWrite(pwm2, 0);
      analogWrite(pwm3, 0);
      analogWrite(pwm4, 0);

      if (laser_interrupt_1 == 1)
      {
        laser = 1;
        flagcase1 = 1;
      }

      if (laser_interrupt_3 == 1)
      {

        laser = 2;
        flagcase2 = 1;
      }

      if (laser_interrupt_4 == 1)
      {
        laser = 3;
        flagcase3 = 1;

      }


      break;


    case 1:    // your hand is close to the sensor
      myservo.write(88);

      while (flagcase1 == 1)
      {



        u2 = digitalRead(15);
        v2 = digitalRead(16);
        w2 = digitalRead(17);
        x2 = digitalRead(18);
        y2 = digitalRead(19);
        z2 = digitalRead(20);
        line2();
        ///////////////////////////////////////////////////
        if ((u2 == 1) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 0))
        {
          analogWrite(pwm2, 100);//28
          digitalWrite(dir2, HIGH);
        }

        if ((u2 == 1) && (v2 == 1) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 0))
        {
          analogWrite(pwm2, 60);//22
          digitalWrite(dir2, HIGH);
        }

        if ((u2 == 0) && (v2 == 1) && (w2 == 1) && (x2 == 0) && (y2 == 0) && (z2 == 0))
        {
          analogWrite(pwm2, 30);
          digitalWrite(dir2, HIGH);
        }

        if ((u2 == 0) && (v2 == 0) && (w2 == 1) && (x2 == 1) && (y2 == 0) && (z2 == 0))
        {
          analogWrite(pwm2, 0);
          digitalWrite(dir2, LOW);
        }

        if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 1) && (y2 == 1) && (z2 == 0))
        {
          analogWrite(pwm2, 30);
          digitalWrite(dir2, LOW);
        }

        if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 1) && (z2 == 1))
        {
          analogWrite(pwm2, 60);
          digitalWrite(dir2, LOW);
        }

        if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 1))
        {
          analogWrite(pwm2, 100);
          digitalWrite(dir2, LOW);
        }
        /////////////////////////////////////////////////////

        analogWrite(pwm4, 100);//65  //150
        analogWrite(pwm3, 105);//70  //155
        // line2();
        digitalWrite(dir3, LOW);
        digitalWrite(dir4, LOW);

        /*    BaseSpeed = 100;
            /////////// Read Values from front and side sensors //////////
            ReadUltrasonicSensorValues();

            /////////// Calculate Average Values ////////////
            Avg_S = (V_S_0 + V_S_1) / 2.0;
            Avg_F = (V_F_0 + V_F_1) / 2.0;

            Wall_Follow(Left);

            line2();
        */
        Serial.println(pwm2);

        junctionCount2 = digitalRead(jPulse2);
        digital_stop_2 = digitalRead(digital_2);

        //////////////////////////////
        junctionCount2 = digitalRead(jPulse2);
        if ((junctionCount2 == 1) && (m == 0))
        {
          n++;
          m = 1;
          //Serial.println(n);111111111
          while (junctionCount2 == 1)
          {
            junctionCount2 = digitalRead(jPulse2);
            if (junctionCount2 == 0)
            {
              m = 0;
            }
          }
        }
        //////////////////////////

        if (n == 2)
        {
          u2 = digitalRead(15);
          v2 = digitalRead(16);
          w2 = digitalRead(17);
          x2 = digitalRead(18);
          y2 = digitalRead(19);
          z2 = digitalRead(20);
          flag1 = 3;
          line2();
          while ((n == 2) && (flag1 == 3) )
          {

            u2 = digitalRead(15);
            v2 = digitalRead(16);
            w2 = digitalRead(17);
            x2 = digitalRead(18);
            y2 = digitalRead(19);
            z2 = digitalRead(20);
            line2();
            Serial.println("tz2_junction");

            digital_stop_2 = digitalRead(digital_2);
            buttonState1x = digitalRead(button1x);
            buttonState2x = digitalRead(button2x);
            buttonState1y = digitalRead(button1y);
            buttonState2y = digitalRead(button2y);
            buttonState1z = digitalRead(button1z);
            buttonState2z = digitalRead(button2z);
            analogWrite(pwm4, 40);//65  //150
            analogWrite(pwm3, 40);//70  //155
            // line2();
            digitalWrite(dir3, LOW);
            digitalWrite(dir4, LOW);

            /*      BaseSpeed = 30;
                  ////////// Read Values from front and side sensors ////////////
                  ReadUltrasonicSensorValues();

                  /////////// Calculate Average Values /////////
                  Avg_S = (V_S_0 + V_S_1) / 2.0;
                  Avg_F = (V_F_0 + V_F_1) / 2.0;

                  Wall_Follow(Left);*/
            laser_interrupt_2 = digitalRead(laser2);
            flag1x == 2;

            if (digital_stop_2 == 1) //&& (flag1x == 2))
            {
              //flag1x = 3;
              flag1 = 4;
              while (flag1 == 4)
              {
                flag1v = 1;

                buttonState1x = digitalRead(button1x);
                buttonState2x = digitalRead(button2x);
                buttonState1y = digitalRead(button1y);
                buttonState2y = digitalRead(button2y);
                buttonState1z = digitalRead(button1z);
                buttonState2z = digitalRead(button2z);



                Serial.println("tz2_stop");
                analogWrite(pwm1, 0);
                analogWrite(pwm2, 0);
                analogWrite(pwm3, 0);
                analogWrite(pwm4, 0);
                // analogWrite(pwm8, 50);
                //              digitalWrite(actuateState, HIGH);

                Serial.println("XXXXXXXXXXXXXXX");
                delay(1000);
                myservo.write(0);

                laser_interrupt_2 = digitalRead(laser2);
                delay(1000);
                digitalWrite(50, HIGH);
                digitalWrite(52, LOW);
                Serial.println("acutate 1");
                laser_interrupt_2 = digitalRead(laser2);

                delay(1000);
                digitalWrite(50, LOW);
                digitalWrite(52, HIGH);
                laser_interrupt_2 = digitalRead(laser2);
                Serial.println("acutate 2");
                //      delay(1000);
                if (flag1v == 1)
                {
                  flag1v = 2;
                  while (flag1v == 2)
                  {
                    laser_interrupt_2 = digitalRead(laser2);
                    flag1n = 2;

                    if ((laser_interrupt_2 == 1) && (flag1n == 2))
                    {
                      Serial.println("detect");
                      flag1 = 6;
                      flag1y = 2;
                      flag1x = 3;
                      flag1n = 3;
                      flag1v = 5;
                      m = 0;
                      n = 0;
                    }
                  }
                }
              }
            }
          }
        }

        ////////////////////////reverse

        if ( (laser_interrupt_2 == 1) && (flag1y == 2))
        {
          flag1y = 3;
          u = digitalRead(41);
          v = digitalRead(43);
          w = digitalRead(45);
          x = digitalRead(47);
          y = digitalRead(49);
          z = digitalRead(51);
          while  (flag1y == 3)
          {
            u = digitalRead(41);
            v = digitalRead(43);
            w = digitalRead(45);
            x = digitalRead(47);
            y = digitalRead(49);
            z = digitalRead(51);
            Serial.println("tz2_reverse");

            /*   BaseSpeed = 100;
               ///////// Read Values from front and side sensors //////////
               ReadUltrasonicSensorValues();

               /////////// Calculate Average Values ///////////
               Avg_S = (V_S_0 + V_S_1) / 2.0;
               Avg_F = (V_F_0 + V_F_1) / 2.0;

               Wall_Follow(Right);*/
            analogWrite(pwm4, 100);//65  //150
            analogWrite(pwm3, 105);//70  //155
            // line2();
            digitalWrite(dir3, HIGH);
            digitalWrite(dir4, HIGH);
            line();
            Serial.println(pwm1);


            // junctionCount3 = digitalRead(jPulse3);
            digital_stop_2 = digitalRead(digital_2);

            //////////////////////////////
            junctionCount3 = digitalRead(jPulse3);
            if ((junctionCount3 == 1) && (t == 0))
            {
              up++;
              t = 1;
              Serial.println(up);
              while (junctionCount3 == 1)
              {
                junctionCount3 = digitalRead(jPulse3);
                if (junctionCount3 == 0)
                {
                  t = 0;
                }
              }
            }
            //////////////////////////
            if (up == 2)
            {
              u = digitalRead(41);
              v = digitalRead(43);
              w = digitalRead(45);
              x = digitalRead(47);
              y = digitalRead(49);
              z = digitalRead(51);
              flag1y = 4;
              line();
              while ((up == 2) && (flag1y == 4) )
              {
                u = digitalRead(41);
                v = digitalRead(43);
                w = digitalRead(45);
                x = digitalRead(47);
                y = digitalRead(49);
                z = digitalRead(51);
                line();
                Serial.println(pwm1);
                Serial.println("tz2_reverse_junction");
                //c = Serial3.read();
                digital_stop_2 = digitalRead(digital_2);

                flag1s = 4;
                /*        BaseSpeed = 40;
                        /////// Read Values from front and side sensors ///////////
                        ReadUltrasonicSensorValues();

                        /////////// Calculate Average Values ////////
                        Avg_S = (V_S_0 + V_S_1) / 2.0;
                        Avg_F = (V_F_0 + V_F_1) / 2.0;

                        Wall_Follow(Right);*/
                analogWrite(pwm4, 40);//65  //150
                analogWrite(pwm3, 40);//70  //155
                line();
                digitalWrite(dir3, HIGH);
                digitalWrite(dir4, HIGH);
                // u = 0;
                if ((digital_stop_2 == 1) && (flag1s == 4))
                {
                  flag1s = 5;
                  while (flag1s == 5)
                  {
                    // flag1x = 2;
                    //flag1y = 5;
                    analogWrite(pwm1, 0);
                    analogWrite(pwm2, 0);
                    analogWrite(pwm3, 0);
                    analogWrite(pwm4, 0);
                    Serial.println("tz2_reverse_Stop");
                    //Serial.println(pwm3);
                    laser_interrupt_1 = digitalRead(laser1);
                    laser_interrupt_2 = digitalRead(laser2);
                    laser_interrupt_3 = digitalRead(laser3);

                    flag5n = 2;

                    flag1 = 0;
                    flag1y = 0;
                    flag1x = 0;
                    flag1n = 0;
                    flag5n = 0;
                    flag1s = 0;
                    Serial.println("de555tect");
                    flagtz = 0;
                    m = 0;
                    t = 0;
                    up = 0;
                    n = 0;
                    laser = 0;
                    flagcase1 = 0;
                    flagmanual = 0;

                  }
                }
              }
            }
          }
        }
      }

      break;

  }

}


