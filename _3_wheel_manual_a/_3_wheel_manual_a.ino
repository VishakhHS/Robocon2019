


//            m1
//            O
//            |
//            |
//            O
//          /   \
//        /       \
//      O           O
//    m2              m3

#include <XBOXRECV.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXRECV Xbox(&Usb);

const int m1 = 2;
const int m2 = 3;
const int m3 = 4;
//const int m4 = 5;
//const int m5 = 6;

const int dir1 = 40;
const int dir2 = 41;
const int dir3 = 42;
//const int dir4 = 43;
//const int dir5 = 44;

//const int LedRed = 14;
//const int LedBlue = 15;
//const int LedGreen = 16;



void setup() {

  pinMode(m1, OUTPUT);   //m1
  pinMode(dir1, OUTPUT);

  pinMode(m2, OUTPUT);   //m2
  pinMode(dir2, OUTPUT);

  pinMode(m3, OUTPUT);   //m3
  pinMode(dir3, OUTPUT);

  //pinMode(m4, OUTPUT);   //m4
  //pinMode(dir4, OUTPUT);

  //pinMode(m5, OUTPUT);   //wheel
  //pinMode(dir5, OUTPUT);

  //pinMode(LedRed, OUTPUT);
  //pinMode(LedBlue, OUTPUT);
  //pinMode(LedGreen, OUTPUT);

  while (1)
  {
    n = 30;
    a = 2 * n;


    analogWrite(m1, a);
    analogWrite(m2, n);
    analogWrite(m3, n);


    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    digitalWrite(dir3, HIGH);

    Serial.println("Left");

  }


  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}
//int a, b, c, d, e, f, j, k, l, m, g, h, n, o, p, q;
int a, b, n, o;
//uint8_t i = 0;


void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected)
  {

    Serial.println("Connected");
    MainWorkFunc();

  }
  else
  {
    Serial.println("Disonnected");
    StopFuncBot();
  }
}
void StopFuncBot() {
  analogWrite(m1, 0);
  analogWrite(m2, 0);
  analogWrite(m3, 0);
  //  analogWrite(m4, 0);
  //  analogWrite(m5, 0);
  Serial.println("Stop");
}



void MainWorkFunc()
{
  //  for (uint8_t i = 0; i < 4; i++) {

  if ((Xbox.Xbox360Connected[0]) || (Xbox.Xbox360Connected[1]) || (Xbox.Xbox360Connected[2]) || (Xbox.Xbox360Connected[3]))
  {
    //n = 0;

    //Backward
    if (Xbox.getButtonPress(L2) > 5)
    {
      // n = Xbox.getButtonPress(L2) + 18;//7
      // if (n > 255)
      // {
      //   n = 255;
      // }
      analogWrite(m1, 0);
      analogWrite(m2, Xbox.getButtonPress(L2));
      analogWrite(m3, Xbox.getButtonPress(L2));
      Serial.println(Xbox.getButtonPress(L2));

      digitalWrite(dir1, LOW);
      digitalWrite(dir2, LOW);
      digitalWrite(dir3, HIGH);
      Serial.println("Backward");

    }

    //Forward
    if (Xbox.getButtonPress(R2) > 5)
    {
      /*  o = Xbox.getButtonPress(R2) + 18;//7
        if (o > 255)
        {
          o = 255;
        }
      */
      analogWrite(m1, 0);
      analogWrite(m2, Xbox.getButtonPress(R2));
      analogWrite(m3, Xbox.getButtonPress(R2));
      Serial.println(Xbox.getButtonPress(R2));

      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
      digitalWrite(dir3, LOW);
      Serial.println("Forward");
    }

    //Left
    if (Xbox.getButtonPress(X))
    {
      n = 30;
      a = 2 * n;


      analogWrite(m1, a);
      analogWrite(m2, n);
      analogWrite(m3, n);


      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
      digitalWrite(dir3, HIGH);

      Serial.println("Left");
    }

    //Right
    if (Xbox.getButtonPress(B))
    {
      o = 30;
      b = 2 * o;

      analogWrite(m1, b);
      analogWrite(m2, o);
      analogWrite(m3, o);


      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
      digitalWrite(dir3, LOW);
      Serial.println("Right");
    }
    //Right Rot
    if (Xbox.getButtonPress(R1))
    {
      analogWrite(m1, 40);
      analogWrite(m2, 40);
      analogWrite(m3, 40);


      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, HIGH);
      digitalWrite(dir3, HIGH);
      Serial.println("Right Rotate");

    }
    //Test
    /*if (Xbox.getButtonPress(Y))
      {
      analogWrite(m1, 255);
      analogWrite(m2, 0);
      analogWrite(m3, 0);


      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, HIGH);
      digitalWrite(dir3, HIGH);
      Serial.println("Right Rotate");

      }
    */
    //Left Rot
    if (Xbox.getButtonPress(L1))
    {
      analogWrite(m1, 40);
      analogWrite(m2, 40);
      analogWrite(m3, 40);


      digitalWrite(dir1, LOW);
      digitalWrite(dir2, LOW);
      digitalWrite(dir3, LOW);

      Serial.println("Left Rotate");
    }
    if ( (Xbox.getButtonPress(L1)) && (Xbox.getButtonPress(R2) > 5))
    {
      analogWrite(m1, 40);
      analogWrite(m2, Xbox.getButtonPress(R2));
      analogWrite(m3, Xbox.getButtonPress(R2));


      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
      digitalWrite(dir3, LOW);

      Serial.println("Forward & Left Rotate");
    }
    if ( (Xbox.getButtonPress(R1)) && (Xbox.getButtonPress(R2) > 5))
    {
      analogWrite(m1, 40);
      analogWrite(m2, Xbox.getButtonPress(R2));
      analogWrite(m3, Xbox.getButtonPress(R2));


      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, HIGH);
      digitalWrite(dir3, LOW);

      Serial.println("Forward & Right Rotate");
    }
    if ( (Xbox.getButtonPress(L1)) && (Xbox.getButtonPress(L2) > 5))
    {
      analogWrite(m1, 40);
      analogWrite(m2, Xbox.getButtonPress(L2));
      analogWrite(m3, Xbox.getButtonPress(L2));


      digitalWrite(dir1, LOW);
      digitalWrite(dir2, LOW);
      digitalWrite(dir3, HIGH);

      Serial.println("Backward & Left Rotate");
    }
    if ( (Xbox.getButtonPress(R1)) && (Xbox.getButtonPress(L2) > 5))
    {
      analogWrite(m1, 40);
      analogWrite(m2, Xbox.getButtonPress(L2));
      analogWrite(m3, Xbox.getButtonPress(L2));


      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
      digitalWrite(dir3, HIGH);

      Serial.println("Backward & Right Rotate");
    }
    /*

            //WheelR
            else if (Xbox.getButtonPress(Y))
            {
              analogWrite(m1, 0);
              analogWrite(m2, 0);
              analogWrite(m3, 0);
              analogWrite(m4, 0);
              analogWrite(m5, 100);

              digitalWrite(dir1, HIGH);
              digitalWrite(dir2, HIGH);
              digitalWrite(dir3, HIGH);
              digitalWrite(dir4, HIGH);
              digitalWrite(dir5, HIGH);

            }

            //WheelL
            else if (Xbox.getButtonPress(A))
            {

               analogWrite(m1, 0);
              analogWrite(m2, 0);
              analogWrite(m3, 0);
              analogWrite(m4, 0);
              analogWrite(m5, 100 );

              digitalWrite(dir1, LOW);
              digitalWrite(dir2, LOW);
              digitalWrite(dir3, LOW);
              digitalWrite(dir4, LOW);
              digitalWrite(dir5, LOW);
            }
            //32767

            //PodForward
            else if (Xbox.getAnalogHat(RightHatY) > 7599)
            {
              a = Xbox.getAnalogHat(RightHatY);
              b = map(a, 7600, 32767, 0, 180);
              p = b +  0;//7
              if (p > 255)
              {
                p = 255;
              }

              analogWrite(m1, 0);
              analogWrite(m2, 0);
              analogWrite(m3, p);
              analogWrite(m4, b);
              analogWrite(m5, 0);

              digitalWrite(dir1, HIGH);
              digitalWrite(dir2, HIGH);
              digitalWrite(dir3, HIGH);
              digitalWrite(dir4, HIGH);
              digitalWrite(dir5, HIGH);
            }

            //PodBackward
            else if (Xbox.getAnalogHat(RightHatY) < -7599)
            {
              c = Xbox.getAnalogHat(RightHatY);
              d = map(c, -7600, -32768, 0, 180);
              q = d + 0;//7
              if (q > 255)
              {
                q = 255;
              }


              analogWrite(m1, 0);
              analogWrite(m2, 0);
              analogWrite(m3, q);
              analogWrite(m4, d);
              analogWrite(m5, 0);

              digitalWrite(dir1, LOW);
              digitalWrite(dir2, LOW);
              digitalWrite(dir3, LOW);
              digitalWrite(dir4, LOW);
              digitalWrite(dir5, LOW);
            }

            //PodRight
            else if (Xbox.getAnalogHat(RightHatX) > 7599)
            {
              e = Xbox.getAnalogHat(RightHatX);
              f = map(e, 7600, 32767, 0, 40);

              analogWrite(m1, f);
              analogWrite(m2, f);
              analogWrite(m3, f);
              analogWrite(m4, f);
              analogWrite(m5, 0);

              digitalWrite(dir1, HIGH);
              digitalWrite(dir2, LOW);
              digitalWrite(dir3, LOW);
              digitalWrite(dir4, HIGH);
              digitalWrite(dir5, HIGH);
            }

            //PodLeft
            else if (Xbox.getAnalogHat(RightHatX) < -7599)
            {
              g = Xbox.getAnalogHat(RightHatX);
              h = map(g, -7600, -32768, 0, 40);

              analogWrite(m1, h);
              analogWrite(m2, h);
              analogWrite(m3, h);
              analogWrite(m4, h);
              analogWrite(m5, 0);

              digitalWrite(dir1, LOW);
              digitalWrite(dir2, HIGH);
              digitalWrite(dir3, HIGH);
              digitalWrite(dir4, LOW);
              digitalWrite(dir5, LOW);
            }

            //LPodForward
            else if (Xbox.getAnalogHat(LeftHatX) > 7599)
            {
              j = Xbox.getAnalogHat(LeftHatX);
              k = map(j, 7600, 32767, 0, 180);

              analogWrite(m1, k);
              analogWrite(m2, k);
              analogWrite(m3, 0);
              analogWrite(m4, 0);
              analogWrite(m5, 0);

              digitalWrite(dir1, LOW);
              digitalWrite(dir2, LOW);
              digitalWrite(dir3, LOW);
              digitalWrite(dir4, LOW);
              digitalWrite(dir5, LOW);
            }

            //LPodBackward
            else if (Xbox.getAnalogHat(LeftHatX) < -7599)
            {
              l = Xbox.getAnalogHat(LeftHatX);
              m = map(l, -7600, -32768, 0, 180);

              analogWrite(m1, m);
              analogWrite(m2, m);
              analogWrite(m3, 0);
              analogWrite(m4, 0);
              analogWrite(m5, 0);

              digitalWrite(dir1, HIGH);
              digitalWrite(dir2, HIGH);
              digitalWrite(dir3, HIGH);
              digitalWrite(dir4, HIGH);
              digitalWrite(dir5, HIGH);
            }

            //Buffer
            else if (((Xbox.getAnalogHat(RightHatY) > 7500) && (Xbox.getAnalogHat(RightHatY) < 7600)) || ((Xbox.getAnalogHat(RightHatY) < -7500) && (Xbox.getAnalogHat(RightHatY) > -7600)) || ((Xbox.getAnalogHat(RightHatX) > 7500) && (Xbox.getAnalogHat(RightHatX) < 7600)) || ((Xbox.getAnalogHat(RightHatX) < -7500) && (Xbox.getAnalogHat(RightHatX) > -7600)) || ((Xbox.getAnalogHat(LeftHatX) < -7500) && (Xbox.getAnalogHat(LeftHatX) > -7600)) || ((Xbox.getAnalogHat(LeftHatX) > 7500) && (Xbox.getAnalogHat(LeftHatX) < 7600)))
            {
              analogWrite(m3, 0);
              analogWrite(m4, 0);
              analogWrite(m1, 0);
              analogWrite(m2, 0);
              analogWrite(m5, 0);

              digitalWrite(dir5, HIGH);
              digitalWrite(dir3, HIGH);
              digitalWrite(dir4, HIGH);
              digitalWrite(dir1, HIGH);
              digitalWrite(dir2, HIGH);
            }
            else if ((Xbox.getButtonPress(UP)))
            {
              digitalWrite(LedRed, HIGH);
            }
            else if ((Xbox.getButtonPress(LEFT)))
            {
              digitalWrite(LedBlue, HIGH);
            }
            else if ((Xbox.getButtonPress(RIGHT)))
            {
              digitalWrite(LedGreen, HIGH);
            }
    */
    else
    {
      analogWrite(m3, 0);
      analogWrite(m2, 0);
      analogWrite(m1, 0);
      // analogWrite(m2, 0);
      // analogWrite(m5, 0);

      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, HIGH);
      digitalWrite(dir3, HIGH);
      // digitalWrite(dir1, HIGH);
      // digitalWrite(dir2, HIGH);

      // digitalWrite(LedRed, LOW);
      // digitalWrite(LedBlue, LOW);
      // digitalWrite(LedGreen, LOW);

    }
  }
  else
  {
    Serial.println("Xbox_Disconnected");

    StopFuncBot();
  }
}

