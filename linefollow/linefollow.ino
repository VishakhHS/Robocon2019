// STARTING POSITION
//                    pwm3
//                      |
//     ^                |
//     |      pwm2 -----O----- pwm1
//     |                |
//                      |
//                    pwm4


int u, v, w, x, y, z;               // back wheel
int u1, v1, w1, x1, y1, z1;         // front wheel
int u2, v2, w2, x2, y2, z2;         // left side wheel
int u3, v3, w3, x3, y3, z3;         // right side wheel

int start = 0;
int flag = 0;
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
int flag4 = 0;
int flag5 = 0;
int flag6 = 0;
int flag7 = 0;
int flag8 = 0;
int flag9 = 0;
int flag10 = 0;
int flag11 = 0;
int flag12 = 0;
int flag13 = 0;
int flag14 = 0;
int flag15 = 0;

const byte dir1 = 3;
const byte dir2 = 9;
const byte dir3 = 11;
const byte dir4 = 5;

const byte pwm1 = 4;
const byte pwm2 = 10;
const byte pwm3 = 12;
const byte pwm4 = 6;

const byte jPulse = A8;   // Connect JPULSE of LSA08 to pin 4
const byte jPulse2 = 14;
const byte jPulse3 = 53;
const byte jPulse4 = 37;

const byte digital_1 = 21;
const byte digital_2 = A15;

const byte laser1 = A4;
const byte laser2 = A7;
const byte laser3 = A6;
const byte laser4 = A5;
int laser = 0;

unsigned int laser_interrupt_1;
unsigned int laser_interrupt_2;
unsigned int laser_interrupt_3;
unsigned int laser_interrupt_4;

void back_line_read()
{
  u = digitalRead(22);
  v = digitalRead(23);
  w = digitalRead(24);
  x = digitalRead(25);
  y = digitalRead(26);
  z = digitalRead(27);
}

void front_line_read()
{
  u1 = digitalRead(28);
  v1 = digitalRead(29);
  w1 = digitalRead(30);
  x1 = digitalRead(31);
  y1 = digitalRead(32);
  z1 = digitalRead(33);
}
void left_line_read()
{
  u2 = digitalRead(34);
  v2 = digitalRead(35);
  w2 = digitalRead(36);
  x2 = digitalRead(37);
  y2 = digitalRead(38);
  z2 = digitalRead(39);
}

void right_line_read()
{
  u3 = digitalRead(40);
  v3 = digitalRead(41);
  w3 = digitalRead(42);
  x3 = digitalRead(43);
  y3 = digitalRead(44);
  z3 = digitalRead(45);
}
void back_line_follow()
{
  if ((u == 1) && (v == 0) && (w == 0) && (x == 0) && (y == 0) && (z == 0))
  {
    analogWrite(pwm4, 150);//22
    digitalWrite(dir4, LOW);
  }


  if ((u == 1) && (v == 1) && (w == 0) && (x == 0) && (y == 0) && (z == 0))
  {
    analogWrite(pwm4, 100);//18
    digitalWrite(dir4, LOW);
  }


  if ((u == 0) && (v == 1) && (w == 1) && (x == 0) && (y == 0) && (z == 0))
  {
    analogWrite(pwm4, 50);//18
    digitalWrite(dir4, LOW);
  }

  if ((u == 0) && (v == 0) && (w == 1) && (x == 1) && (y == 0) && (z == 0))
  {
    analogWrite(pwm4, 0);
    digitalWrite(dir4, LOW);
  }

  if ((u == 0) && (v == 0) && (w == 0) && (x == 1) && (y == 1) && (z == 0))
  {
    analogWrite(pwm4, 50);
    digitalWrite(dir4, HIGH);
  }

  if ((u == 0) && (v == 0) && (w == 0) && (x == 0) && (y == 1) && (z == 1))
  {
    analogWrite(pwm4, 100);
    digitalWrite(dir4, HIGH);
  }

  if ((u == 0) && (v == 0) && (w == 0) && (x == 0) && (y == 0) && (z == 1))
  {
    analogWrite(pwm4, 150);
    digitalWrite(dir4, HIGH);
  }
}


void front_line_follow()
{
  if ((u1 == 1) && (v1 == 0) && (w1 == 0) && (x1 == 0) && (y1 == 0) && (z1 == 0))
  {
    analogWrite(pwm3, 150);
    digitalWrite(dir3, LOW);
  }


  if ((u1 == 1) && (v1 == 1) && (w1 == 0) && (x1 == 0) && (y1 == 0) && (z1 == 0))
  {
    analogWrite(pwm3, 100);
    digitalWrite(dir3, LOW);
  }


  if ((u1 == 0) && (v1 == 1) && (w1 == 1) && (x1 == 0) && (y1 == 0) && (z1 == 0))
  {
    analogWrite(pwm3, 50);
    digitalWrite(dir3, LOW);
  }

  if ((u1 == 0) && (v1 == 0) && (w1 == 1) && (x1 == 1) && (y1 == 0) && (z1 == 0))
  {
    analogWrite(pwm3, 0);
    digitalWrite(dir3, LOW);
  }

  if ((u1 == 0) && (v1 == 0) && (w1 == 0) && (x1 == 1) && (y1 == 1) && (z1 == 0))
  {
    analogWrite(pwm3, 50);
    digitalWrite(dir3, HIGH);
  }

  if ((u1 == 0) && (v1 == 0) && (w1 == 0) && (x1 == 0) && (y1 == 1) && (z1 == 1))
  {
    analogWrite(pwm3, 100);
    digitalWrite(dir3, HIGH);
  }

  if ((u1 == 0) && (v1 == 0) && (w1 == 0) && (x1 == 0) && (y1 == 0) && (z1 == 1))
  {
    analogWrite(pwm3, 150);
    digitalWrite(dir3, HIGH);
  }
}

void left_line_follow()
{
  if ((u2 == 1) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 0))
  {
    analogWrite(pwm2, 150);
    digitalWrite(dir2, LOW);
  }


  if ((u2 == 1) && (v2 == 1) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 0))
  {
    analogWrite(pwm2, 100);
    digitalWrite(dir2, LOW);
  }


  if ((u2 == 0) && (v2 == 1) && (w2 == 1) && (x2 == 0) && (y2 == 0) && (z2 == 0))
  {
    analogWrite(pwm2, 50);
    digitalWrite(dir2, LOW);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 1) && (x2 == 1) && (y2 == 0) && (z2 == 0))
  {
    analogWrite(pwm2, 0);
    digitalWrite(dir2, LOW);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 1) && (y2 == 1) && (z2 == 0))
  {
    analogWrite(pwm2, 50);
    digitalWrite(dir2, HIGH);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 1) && (z2 == 1))
  {
    analogWrite(pwm2, 100);
    digitalWrite(dir2, HIGH);
  }

  if ((u2 == 0) && (v2 == 0) && (w2 == 0) && (x2 == 0) && (y2 == 0) && (z2 == 1))
  {
    analogWrite(pwm2, 150);
    digitalWrite(dir2, HIGH);
  }
}

void right_line_follow()
{
  if ((u3 == 1) && (v3 == 0) && (w3 == 0) && (x3 == 0) && (y3 == 0) && (z3 == 0))
  {
    analogWrite(pwm1, 150);
    digitalWrite(dir1, LOW);
  }


  if ((u3 == 1) && (v3 == 1) && (w3 == 0) && (x3 == 0) && (y3 == 0) && (z3 == 0))
  {
    analogWrite(pwm1, 100);//18
    digitalWrite(dir1, LOW);
  }


  if ((u3 == 0) && (v3 == 1) && (w3 == 1) && (x3 == 0) && (y3 == 0) && (z3 == 0))
  {
    analogWrite(pwm1, 50);
    digitalWrite(dir1, LOW);
  }

  if ((u3 == 0) && (v3 == 0) && (w3 == 1) && (x3 == 1) && (y3 == 0) && (z3 == 0))
  {
    analogWrite(pwm1, 0);
    digitalWrite(dir1, LOW);
  }

  if ((u3 == 0) && (v3 == 0) && (w3 == 0) && (x3 == 1) && (y3 == 1) && (z3 == 0))
  {
    analogWrite(pwm1, 50);
    digitalWrite(dir1, HIGH);
  }

  if ((u3 == 0) && (v3 == 0) && (w3 == 0) && (x3 == 0) && (y3 == 1) && (z3 == 1))
  {
    analogWrite(pwm1, 100);
    digitalWrite(dir1, HIGH);
  }

  if ((u3 == 0) && (v3 == 0) && (w3 == 0) && (x3 == 0) && (y3 == 0) && (z3 == 1))
  {
    analogWrite(pwm1, 150);
    digitalWrite(dir1, HIGH);
  }
}



void setup() {
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
  pinMode(7, OUTPUT);

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

  pinMode(digital_1, INPUT);
  pinMode(digital_2, INPUT);


  pinMode(laser1, INPUT);
  pinMode(laser2, INPUT);
  pinMode(laser3, INPUT);
  pinMode(laser4, INPUT);

  Serial.begin(9600);
  while (flag == 1)
  {
    analogWrite(pwm1, 100);
    analogWrite(pwm2, 100);
    digitalWrite(pwm1, HIGH);
    digitalWrite(pwm2, LOW);
    front_line_read();
    front_line_follow();
    junctionCount = digitalRead(jPulse);
    if ((junctionCount == 1) && (flag == 1))                   //Front_Slow
    {
      flag == 2;
      while (flag == 2)
      {
        analogWrite(pwm1, 50);
        analogWrite(pwm2, 50);
        digitalWrite(pwm1, HIGH);
        digitalWrite(pwm2, LOW);
        front_line_read();
        front_line_follow();

        digital_stop_1 = digitalRead(digital_1);
        if (digital_stop_1 == 1)                                //Stop at TZ1 set point
        {
          flag = 3;

          analogWrite(pwm1, 0);
          analogWrite(pwm2, 0);
          analogWrite(pwm3, 0);
          analogWrite(pwm4, 0);

        }
      }

    }
  }




}

void loop() {







  while (start == 0)
  {
    laser_interrupt_1 = digitalRead(laser1);
    laser_interrupt_2 = digitalRead(laser2);
    laser_interrupt_3 = digitalRead(laser3);
    flag1 = 0;
    flag2 = 0;
    flag3 = 0;
    if (laser_interrupt_1 == 1 && start == 0)
    {
      laser = 1;
      flag1 = 1;
    }
    if (laser_interrupt_2 == 1 && start == 0)
    {
      laser = 2;
      flag2 = 1;
    }
    if (laser_interrupt_3 == 1 && start == 0)
    {
      laser = 3;
      flag3 = 1;
    }

    switch (laser)
    {
      case 1:
        while (flag1 == 1)
        {
          analogWrite(pwm4, 100);
          analogWrite(pwm3, 105);

          digitalWrite(dir3, LOW);
          digitalWrite(dir4, LOW);
          left_line_follow();
          left_line_read();
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
        }

      case 2:
        while (flag2 == 1)
        {

        }

      case 3:
        while (flag3 == 1)
        {

        }
    }

  }



}
