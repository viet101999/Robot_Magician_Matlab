
/* Analog and Digital Input and Output Server for MATLAB     */
/* Giampiero Campa, Copyright 2012 The MathWorks, Inc        */

/* This file is meant to be used with the MATLAB arduino IO
   package, however, it can be used from the IDE environment
   (or any other serial terminal) by typing commands like:

   0e0 : assigns digital pin #4 (e) as input
   0f1 : assigns digital pin #5 (f) as output
   0n1 : assigns digital pin #13 (n) as output

   1c  : reads digital pin #2 (c)
   1e  : reads digital pin #4 (e)
   2n0 : sets digital pin #13 (n) low
   2n1 : sets digital pin #13 (n) high
   2f1 : sets digital pin #5 (f) high
   2f0 : sets digital pin #5 (f) low
   4j2 : sets digital pin #9 (j) to  50=ascii(2) over 255
   4jz : sets digital pin #9 (j) to 122=ascii(z) over 255
   3a  : reads analog pin #0 (a)
   3f  : reads analog pin #5 (f)

   5j    : reads status (attached/detached) of servo on pin #9
   5k    : reads status (attached/detached) of servo on pin #10
   6j1   : attaches servo on pin #9
   8jz   : moves servo on pin #9 of 122 degrees (122=ascii(z))
   7j    : reads angle of servo on pin #9
   6j0   : detaches servo on pin #9

   E0cd  : attaches encoder #0 (0) on pins 2 (c) and 3 (d)
   E1st  : attaches encoder #1 on pins 18 (s) and 19 (t)
   E2vu  : attaches encoder #2 on pins 21 (v) and 20 (u)
   G0    : gets 0 position of encoder #0
   I0u   : sets debounce delay to 20 (2ms) for encoder #0
   H1    : resets position of encoder #1
   F2    : detaches encoder #2

   R0    : sets analog reference to DEFAULT
   R1    : sets analog reference to INTERNAL
   R2    : sets analog reference to EXTERNAL

   X3    : roundtrip example case returning the input (ascii(3))
   99    : returns script type (0 adio.pde ... 3 motor.pde ) */

#include <Servo.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <TimerFour.h>
#include <TimerFive.h>

/* define internal for the MEGA as 1.1V (as as for the 328)  */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define INTERNAL INTERNAL1V1
#endif

/* define encoder structure                                  */
typedef struct {
  int pinA;
  int pinB;
  int pos;
  int del;
} Encoder;
volatile Encoder Enc[3] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

/* create servo vector                                       */
Servo servo[70];

// ***********define STEP***********
#define servoPin1 5
#define servoPin2 6
#define servoPin3 7

const int stepPin1 = 24;
const int dirPin1 = 2;
const int enPin1 = 22;

const int stepPin2 = 28;
const int dirPin2 = 3;
const int enPin2 = 26;

const int stepPin3 = 32;
const int dirPin3 = 4;
const int enPin3 = 30;

long v1 = 0, v2 = 0, v3 = 0;
float the1pre, the2pre, the3pre;
float the1_flex, the2_flex, the3_flex;
float the1, the2, the3;
float tc = 0, t_int;
float a1, a2, a3, b1, b2, b3, c1, c2, c3, d1, d2, d3;
float num1, num2, num3;
float x1, y1, z1, x2, y2, z2, x3, y3, z3;
float vtmm1 , vtmm2 , vtmm3;
int a = 0, b = 0, c = 0;
float vt1 = 90 * 200 / 9, vt2 = 118 * 200 / 9, vt3 = 157 * 200 / 9, v3_pre_nhap = vt3;
int flag, i, j;
int theta1, theta2, theta3, tg;

void setup()
{
  /* initialize serial                                       */
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  vtmm1 = 0;
  vtmm2 = 0;
  vtmm3 = 0;

  pinMode(servoPin1, OUTPUT);
  pinMode(servoPin2, OUTPUT);
  pinMode(servoPin3, OUTPUT);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  digitalWrite(enPin1, LOW);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);
  digitalWrite(enPin2, LOW);

  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(enPin3, OUTPUT);
  digitalWrite(enPin3, LOW);

  Timer5.initialize(100000);
  Timer5.attachInterrupt(ab); //ngắt bằng chương trì  nh ab
}

void ab()
{
  //Serial.println("ab()");
  t_int++;
  the1pre = the1_flex;
  the2pre = the2_flex;
  the3pre = the3_flex;
  // tinh toan qui hoach qui dao va dong toc do
  if (t_int <= (tc * 10))
  { // tính toán qui hoạch quỹ đạo
    //Serial.println("quy dao");
    the1_flex = a1 + b1 * (t_int / 10) + c1 * pow(t_int / 10, 2) + d1 * pow(t_int / 10, 3);
    the2_flex = a2 + b2 * (t_int / 10) + c2 * pow(t_int / 10, 2) + d2 * pow(t_int / 10, 3);
    the3_flex = a3 + b3 * (t_int / 10) + c3 * pow(t_int / 10, 2) + d3 * pow(t_int / 10, 3);
    a = 0; b = 0; c = 0; // đồng tốc độ step trong 200ms
    if (the1_flex - the1pre == 0) v1 = 100000;
    else v1 = (100000 / abs(the1_flex - the1pre));
    if (the2_flex - the2pre == 0) v2 = 100000;
    else v2 = (100000 / abs(the2_flex - the2pre));
    if (the3_flex - the3pre == 0) v3 = 100000;
    else v3 = (100000 / abs(the3_flex - the3pre));
    //Serial.println(the1_flex);
    //Serial.println(the2_flex);
    //Serial.println(the3_flex);

    // khai báo timer
    Timer1.initialize(v1);
    Timer1.attachInterrupt(step1);
    Timer3.initialize(v2);
    Timer3.attachInterrupt(step2);
    Timer4.initialize(v3);
    Timer4.attachInterrupt(step3);
    Timer1.start();
    Timer3.start();
    Timer4.start();
    //Serial.println("ab()_last");
  }
  if (t_int > (tc * 10))
  {
    Timer5.stop();
  }
}

void step1()
{
  //Serial.println("step1");
  if ((the1_flex - the1pre) > 0) // so sánh để chọn chiều quay step
  {
    digitalWrite(dirPin1, HIGH); // cho phep step di theo chieu qui dinh
    if (a <= the1_flex - the1pre)
    {
      digitalWrite(stepPin1, !digitalRead(stepPin1));
      a++;
      vt1++;
    }
    else
    {
      Timer1.stop();
    }
  }
  else
  {
    digitalWrite(dirPin1, LOW); // cho phep step di theo chieu qui dinh
    if (a <= abs(the1_flex - the1pre))
    {
      digitalWrite(stepPin1, !digitalRead(stepPin1));
      a++;
      vt1--;
    }
    else
    {
      Timer1.stop();
    }
  }
}

void step2()
{
  //Serial.println("step1");
  if ((the2_flex - the2pre) > 0) // so sánh để chọn chiều quay step
  {
    digitalWrite(dirPin2, LOW); // cho phep step di theo chieu qui dinh
    if (b <= the2_flex - the2pre)
    {
      digitalWrite(stepPin2, !digitalRead(stepPin2));
      b++;
      vt2++;
    }
    else
    {
      Timer3.stop();
    }
  }
  else
  {
    digitalWrite(dirPin2, HIGH); // cho phep step di theo chieu qui dinh
    if (b <= abs(the2_flex - the2pre))
    {
      digitalWrite(stepPin2, !digitalRead(stepPin2));
      b++;
      vt2--;
    }
    else
    {
      Timer3.stop();
    }
  }
}
void step3()
{
  //Serial.println("step1");
  if (the3_flex - the3pre > 0) // so sánh để chọn chiều quay step
  {
    digitalWrite(dirPin3, LOW); // cho phep step di theo chieu qui dinh
    if (c <= the3_flex - the3pre)
    {
      digitalWrite(stepPin3, !digitalRead(stepPin3));
      c++;
      vt3++;
    }
    else
    {
      Timer4.stop();
    }
  }
  else
  {
    digitalWrite(dirPin3, HIGH); // cho phep step di theo chieu qui dinh
    if (c <= abs(the3_flex - the3pre))
    {
      digitalWrite(stepPin3, !digitalRead(stepPin3));
      c++;
      vt3--;
    }
    else
    {
      Timer4.stop();
    }
  }
}

void robot(int the1, float the2, float the3, float tgian)
{
  //Serial.println("robot_function");
  float v2_delta = 0.0, v3_delta = 0.0;
  y1 = the1;
  y2 = the2;
  y3 = the3;
  tc = tgian;

  v2_delta = vt2 - (y2 * 200 / 9);
  v3_delta = (y3 * 200 / 9) - v3_pre_nhap;
  v3_pre_nhap = (y3 * 200 / 9);

  //Serial.println(y1);
  //Serial.println(y2);
  //Serial.println(y3);
  //Serial.println(tc);

  // tính toán số xung cần thiết của step
  z1 = y1 * 200 / 9;
  z2 = y2 * 200 / 9;
  z3 = vt3 + v2_delta + v3_delta;
  v2_delta = 0.0;
  v3_delta = 0.0;
  // tính số xung cần thiết quay cho step
  vtmm1 = z1 - vt1;
  vtmm2 = z2 - vt2;
  vtmm3 = z3 - vt3;
  Serial.println(z1*9/200);
  Serial.println(z2*9/200);
  Serial.println(z3*9/200);
  a = 0; b = 0; c = 0;

  // tính toán hệ số cho việc qui hoạch quỹ đạo
  the1 = vtmm1;
  the2 = vtmm2;
  the3 = vtmm3;
  the1pre = 0, the2pre = 0, the3pre = 0; t_int = 0;
  the1_flex = 0, the2_flex = 0, the3_flex = 0;

  a1 = the1pre;
  b1 = 0;
  c1 = 3 * (the1 - the1pre) / pow(tc, 2);
  d1 = -2 * (the1 - the1pre) / pow(tc, 3);

  a2 = the2pre;
  b2 = 0;
  c2 = 3 * (the2 - the2pre) / pow(tc, 2);
  d2 = -2 * (the2 - the2pre) / pow(tc, 3);

  a3 = the3pre;
  b3 = 0;
  c3 = 3 * (the3 - the3pre) / pow(tc, 2);
  d3 = -2 * (the3 - the3pre) / pow(tc, 3);

  Timer5.start();
}

void loop()
{

  /* variables declaration and initialization                */

  static int  s   = -1;    /* state                          */
  static int  pin = 13;    /* generic pin number             */
  static int  enc = 0;     /* generic encoder number         */
  static int  en_robot = 0;

  int  val =  0;           /* generic value read from serial */
  int  agv =  0;           /* generic analog value           */
  int  dgv =  0;           /* generic digital value          */


  /* The following instruction constantly checks if anything
     is available on the serial port. Nothing gets executed in
     the loop if nothing is available to be read, but as soon
     as anything becomes available, then the part coded after
     the if statement (that is the real stuff) gets executed */

  if (Serial.available() > 0)
  {

    /* whatever is available from the serial is read here    */
    val = Serial.read();

    /* This part basically implements a state machine that
       reads the serial port and makes just one transition
       to a new state, depending on both the previous state
       and the command that is read from the serial port.
       Some commands need additional inputs from the serial
       port, so they need 2 or 3 state transitions (each one
       happening as soon as anything new is available from
       the serial port) to be fully executed. After a command
       is fully executed the state returns to its initial
       value s=-1                                            */

    switch (s)
    {


      /* s=-1 means NOTHING RECEIVED YET ******************* */
      case -1:

        /* calculate next state                                */
        if (val > 47 && val < 90)
        {
          /* the first received value indicates the mode
                 49 is ascii for 1, ... 90 is ascii for Z
                 s=0 is change-pin mode;
                 s=10 is DI;  s=20 is DO;  s=30 is AI;  s=40 is AO;
                 s=50 is servo status; s=60 is aervo attach/detach;
                 s=70 is servo read;   s=80 is servo write;
                 s=90 is query script type (1 basic, 2 motor);
                 s=210 is encoder attach; s=220 is encoder detach;
                 s=230 is get encoder position; s=240 is encoder reset;
                 s=250 is set encoder debounce delay;
                 s=340 is change analog reference;
                 s=380 robot
                 s=400 example echo returning the input argument;
          */
          s = 10 * (val - 48);
        }

        /* the following statements are needed to handle
           unexpected first values coming from the serial (if
           the value is unrecognized then it defaults to s=-1) */
        if ((s > 90 && s < 210) || (s > 250 && s != 340 && s != 380 && s != 400)) {
          s = -1;
        }

        /* the break statements gets out of the switch-case, so
          /* we go back and wait for new serial data             */
        break; /* s=-1 (initial state) taken care of           */



      /* s=0 or 1 means CHANGE PIN MODE                      */

      case 0:
        /* the second received value indicates the pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          s = 1; /* next we will need to get 0 or 1 from serial  */
        }
        else {
          s = -1; /* if value is not a pin then return to -1     */
        }
        break; /* s=0 taken care of                            */


      case 1:
        /* the third received value indicates the value 0 or 1 */
        if (val > 47 && val < 50) {
          /* set pin mode                                      */
          if (val == 48) {
            pinMode(pin, INPUT);
          }
          else {
            pinMode(pin, OUTPUT);
          }
        }
        s = -1; /* we are done with CHANGE PIN so go to -1      */
        break; /* s=1 taken care of                            */



      /* s=10 means DIGITAL INPUT ************************** */

      case 10:
        /* the second received value indicates the pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          dgv = digitalRead(pin);    /* perform Digital Input  */
          Serial.println(dgv);       /* send value via serial  */
        }
        s = -1; /* we are done with DI so next state is -1      */
        break; /* s=10 taken care of                           */



      /* s=20 or 21 means DIGITAL OUTPUT ******************* */

      case 20:
        /* the second received value indicates the pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          s = 21; /* next we will need to get 0 or 1 from serial */
        }
        else {
          s = -1; /* if value is not a pin then return to -1     */
        }
        break; /* s=20 taken care of                           */

      case 21:
        /* the third received value indicates the value 0 or 1 */
        if (val > 47 && val < 50) {
          dgv = val - 48;            /* calculate value        */
          digitalWrite(pin, dgv);    /* perform Digital Output */
        }
        s = -1; /* we are done with DO so next state is -1      */
        break; /* s=21 taken care of                           */



      /* s=30 means ANALOG INPUT *************************** */

      case 30:
        /* the second received value indicates the pin
           from abs('a')=97, pin 0, to abs('p')=112, pin 15    */
        if (val > 96 && val < 113) {
          pin = val - 97;            /* calculate pin          */
          agv = analogRead(pin);     /* perform Analog Input   */
          Serial.println(agv);       /* send value via serial  */
        }
        s = -1; /* we are done with AI so next state is -1      */
        break; /* s=30 taken care of                           */



      /* s=40 or 41 means ANALOG OUTPUT ******************** */

      case 40:
        /* the second received value indicates the pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          s = 41; /* next we will need to get value from serial  */
        }
        else {
          s = -1; /* if value is not a pin then return to -1     */
        }
        break; /* s=40 taken care of                           */


      case 41:
        /* the third received value indicates the analog value */
        analogWrite(pin, val);       /* perform Analog Output  */
        s = -1; /* we are done with AO so next state is -1      */
        break; /* s=41 taken care of                           */



      /* s=50 means SERVO STATUS (ATTACHED/DETACHED) ******* */

      case 50:
        /* the second value indicates the servo attachment pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          dgv = servo[pin].attached();          /* read status */
          Serial.println(dgv);       /* send value via serial  */
        }
        s = -1; /* we are done with servo status so return to -1*/
        break; /* s=50 taken care of                           */



      /* s=60 or 61 means SERVO ATTACH/DETACH ************** */

      case 60:
        /* the second value indicates the servo attachment pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          s = 61; /* next we will need to get 0 or 1 from serial */
        }
        else {
          s = -1; /* if value is not a servo then return to -1   */
        }
        break; /* s=60 taken care of                           */


      case 61:
        /* the third received value indicates the value 0 or 1
           0 for detach and 1 for attach                       */
        if (val > 47 && val < 50) {
          dgv = val - 48;            /* calculate value        */
          if (dgv) servo[pin].attach(pin);     /* attach servo */
          else servo[pin].detach();            /* detach servo */
        }
        s = -1; /* we are done with servo attach/detach so -1   */
        break; /* s=61 taken care of                           */



      /* s=70 means SERVO READ ***************************** */

      case 70:
        /* the second value indicates the servo attachment pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          agv = servo[pin].read();   /* read value             */
          Serial.println(agv);       /* send value via serial  */
        }
        s = -1; /* we are done with servo read so go to -1 next */
        break; /* s=70 taken care of                           */



      /* s=80 or 81 means SERVO WRITE   ******************** */

      case 80:
        /* the second value indicates the servo attachment pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          s = 81; /* next we will need to get value from serial  */
        }
        else {
          s = -1; /* if value is not a servo then return to -1   */
        }
        break; /* s=80 taken care of                           */


      case 81:
        /* the third received value indicates the servo angle  */
        servo[pin].write(val);                  /* write value */
        s = -1; /* we are done with servo write so go to -1 next*/
        break; /* s=81 taken care of                           */



      /* s=90 means Query Script Type:
         (0 adio, 1 adioenc, 2 adiosrv, 3 motor)             */

      case 90:
        if (val == 57) {
          /* if string sent is 99  send script type via serial */
          Serial.println(2);
        }
        s = -1; /* we are done with this so next state is -1    */
        break; /* s=90 taken care of                           */



      /* s=210 to 212 means ENCODER ATTACH ***************** */

      case 210:
        /* the second value indicates the encoder number:
           either 0, 1 or 2                                    */
        if (val > 47 && val < 51) {
          enc = val - 48;    /* calculate encoder number       */
          s = 211; /* next we need the first attachment pin     */
        }
        else {
          s = -1; /* if value is not an encoder then return to -1*/
        }
        break; /* s=210 taken care of                          */


      case 211:
        /* the third received value indicates the first pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          Enc[enc].pinA = pin;       /* set pin A              */
          s = 212; /* next we need the second attachment pin    */
        }
        else {
          s = -1; /* if value is not a servo then return to -1   */
        }
        break; /* s=211 taken care of                          */


      case 212:
        /* the fourth received value indicates the second pin
           from abs('c')=99, pin 2, to abs('¦')=166, pin 69    */
        if (val > 98 && val < 167) {
          pin = val - 97;            /* calculate pin          */
          Enc[enc].pinB = pin;       /* set pin B              */

          /* set encoder pins as inputs                        */
          pinMode(Enc[enc].pinA, INPUT);
          pinMode(Enc[enc].pinB, INPUT);

          /* turn on pullup resistors                          */
          digitalWrite(Enc[enc].pinA, HIGH);
          digitalWrite(Enc[enc].pinB, HIGH);

          /* attach interrupts                                 */
          switch (enc) {
            case 0:
              attachInterrupt(getIntNum(Enc[0].pinA), isrPinAEn0, CHANGE);
              attachInterrupt(getIntNum(Enc[0].pinB), isrPinBEn0, CHANGE);
              break;
            case 1:
              attachInterrupt(getIntNum(Enc[1].pinA), isrPinAEn1, CHANGE);
              attachInterrupt(getIntNum(Enc[1].pinB), isrPinBEn1, CHANGE);
              break;
            case 2:
              attachInterrupt(getIntNum(Enc[2].pinA), isrPinAEn2, CHANGE);
              attachInterrupt(getIntNum(Enc[2].pinB), isrPinBEn2, CHANGE);
              break;
          }

        }
        s = -1; /* we are done with encoder attach so -1         */
        break; /* s=212 taken care of                          */


      /* s=220 means ENCODER DETACH  *********************** */

      case 220:
        /* the second value indicates the encoder number:
           either 0, 1 or 2                                    */
        if (val > 47 && val < 51) {
          enc = val - 48;    /* calculate encoder number       */
          /* detach interrupts */
          detachInterrupt(getIntNum(Enc[enc].pinA));
          detachInterrupt(getIntNum(Enc[enc].pinB));
        }
        s = -1; /* we are done with encoder detach so -1        */
        break; /* s=220 taken care of                          */


      /* s=230 means GET ENCODER POSITION ****************** */

      case 230:
        /* the second value indicates the encoder number:
           either 0, 1 or 2                                    */
        if (val > 47 && val < 51) {
          enc = val - 48;    /* calculate encoder number       */
          /* send the value back                               */
          Serial.println(Enc[enc].pos);
        }
        s = -1; /* we are done with encoder detach so -1        */
        break; /* s=230 taken care of                          */


      /* s=240 means RESET ENCODER POSITION **************** */

      case 240:
        /* the second value indicates the encoder number:
           either 0, 1 or 2                                    */
        if (val > 47 && val < 51) {
          enc = val - 48;    /* calculate encoder number       */
          /* reset position                                    */
          Enc[enc].pos = 0;
        }
        s = -1; /* we are done with encoder detach so -1        */
        break; /* s=240 taken care of                          */


      /* s=250 and 251 mean SET ENCODER DEBOUNCE DELAY ***** */

      case 250:
        /* the second value indicates the encoder number:
           either 0, 1 or 2                                    */
        if (val > 47 && val < 51) {
          enc = val - 48;    /* calculate encoder number       */
          s = 251; /* next we need the first attachment pin     */
        }
        else {
          s = -1; /* if value is not an encoder then return to -1*/
        }
        break; /* s=250 taken care of                          */


      case 251:
        /* the third received value indicates the debounce
           delay value in units of approximately 0.1 ms each
           from abs('a')=97, 0 units, to abs('¦')=166, 69 units*/
        if (val > 96 && val < 167) {
          Enc[enc].del = val - 97;   /* set debounce delay     */
        }
        s = -1; /* we are done with this so next state is -1    */
        break; /* s=251 taken care of                          */



      /* s=340 or 341 means ANALOG REFERENCE *************** */

      case 340:
        /* the second received value indicates the reference,
           which is encoded as is 0,1,2 for DEFAULT, INTERNAL
           and EXTERNAL, respectively. Note that this function
           is ignored for boards not featuring AVR or PIC32    */

#if defined(__AVR__) || defined(__PIC32MX__)

        switch (val) {

          case 48:
            analogReference(DEFAULT);
            break;

          case 49:
            analogReference(INTERNAL);
            break;

          case 50:
            analogReference(EXTERNAL);
            break;

          default:                 /* unrecognized, no action  */
            break;
        }

#endif

        s = -1; /* we are done with this so next state is -1    */
        break; /* s=341 taken care of                          */

      case 380:
        theta1 = val - 48;
        //Serial.println(the1);
        s = 381;
        break;

      case 381:
        theta2 = val - 48;
        //Serial.println(the2);
        s = 382;
        break;

      case 382:
        theta3 = val - 48;
        //Serial.println(the3);
        s = 383;
        break;

      case 383:
        tg = val - 48;
        //Serial.println(tg);
        en_robot = 1;
        s = -1;
        break;

      /* s=400 roundtrip example function (returns the input)*/

      case 400:
        /* the second value (val) can really be anything here  */

        /* This is an auxiliary function that returns the ASCII
           value of its first argument. It is provided as an
           example for people that want to add their own code  */

        /* your own code goes here instead of the serial print */
        Serial.println(val);

        s = -1; /* we are done with the aux function so -1      */
        break; /* s=400 taken care of                          */

      /* ******* UNRECOGNIZED STATE, go back to s=-1 ******* */

      default:
        /* we should never get here but if we do it means we
           are in an unexpected state so whatever is the second
           received value we get out of here and back to s=-1  */

        s = -1; /* go back to the initial state, break unneeded */



    } /* end switch on state s                               */

    if (en_robot == 1)
    {
      robot(theta1, theta2, theta3, tg);
      en_robot = 0;
    }

  } /* end if serial available                               */

} /* end loop statement                                      */




/* auxiliary function to handle encoder attachment           */
int getIntNum(int pin) {
  /* returns the interrupt number for a given interrupt pin
     see http://arduino.cc/it/Reference/AttachInterrupt        */
  switch (pin) {
    case 2:
      return 0;
    case 3:
      return 1;
    case 21:
      return 2;
    case 20:
      return 3;
    case 19:
      return 4;
    case 18:
      return 5;
    default:
      return -1;
  }
}


/* auxiliary debouncing function                             */
void debounce(int del) {
  int k;
  for (k = 0; k < del; k++) {
    /* can't use delay in the ISR so need to waste some time
       perfoming operations, this uses roughly 0.1ms on uno  */
    k = k + 0.0 + 0.0 - 0.0 + 3.0 - 3.0;
  }
}


/* Interrupt Service Routine: change on pin A for Encoder 0  */
void isrPinAEn0() {

  /* read pin B right away                                   */
  int drB = digitalRead(Enc[0].pinB);

  /* possibly wait before reading pin A, then read it        */
  debounce(Enc[0].del);
  int drA = digitalRead(Enc[0].pinA);

  /* this updates the counter                                */
  if (drA == HIGH) {   /* low->high on A? */

    if (drB == LOW) {  /* check pin B */
      Enc[0].pos++;  /* going clockwise: increment         */
    } else {
      Enc[0].pos--;  /* going counterclockwise: decrement  */
    }

  } else {                       /* must be high to low on A */

    if (drB == HIGH) { /* check pin B */
      Enc[0].pos++;  /* going clockwise: increment         */
    } else {
      Enc[0].pos--;  /* going counterclockwise: decrement  */
    }

  } /* end counter update                                    */

} /* end ISR pin A Encoder 0                                 */



/* Interrupt Service Routine: change on pin B for Encoder 0  */
void isrPinBEn0() {

  /* read pin A right away                                   */
  int drA = digitalRead(Enc[0].pinA);

  /* possibly wait before reading pin B, then read it        */
  debounce(Enc[0].del);
  int drB = digitalRead(Enc[0].pinB);

  /* this updates the counter                                */
  if (drB == HIGH) {   /* low->high on B? */

    if (drA == HIGH) { /* check pin A */
      Enc[0].pos++;  /* going clockwise: increment         */
    } else {
      Enc[0].pos--;  /* going counterclockwise: decrement  */
    }

  } else {                       /* must be high to low on B */

    if (drA == LOW) {  /* check pin A */
      Enc[0].pos++;  /* going clockwise: increment         */
    } else {
      Enc[0].pos--;  /* going counterclockwise: decrement  */
    }

  } /* end counter update */

} /* end ISR pin B Encoder 0  */


/* Interrupt Service Routine: change on pin A for Encoder 1  */
void isrPinAEn1() {

  /* read pin B right away                                   */
  int drB = digitalRead(Enc[1].pinB);

  /* possibly wait before reading pin A, then read it        */
  debounce(Enc[1].del);
  int drA = digitalRead(Enc[1].pinA);

  /* this updates the counter                                */
  if (drA == HIGH) {   /* low->high on A? */

    if (drB == LOW) {  /* check pin B */
      Enc[1].pos++;  /* going clockwise: increment         */
    } else {
      Enc[1].pos--;  /* going counterclockwise: decrement  */
    }

  } else { /* must be high to low on A                       */

    if (drB == HIGH) { /* check pin B */
      Enc[1].pos++;  /* going clockwise: increment         */
    } else {
      Enc[1].pos--;  /* going counterclockwise: decrement  */
    }

  } /* end counter update                                    */

} /* end ISR pin A Encoder 1                                 */


/* Interrupt Service Routine: change on pin B for Encoder 1  */
void isrPinBEn1() {

  /* read pin A right away                                   */
  int drA = digitalRead(Enc[1].pinA);

  /* possibly wait before reading pin B, then read it        */
  debounce(Enc[1].del);
  int drB = digitalRead(Enc[1].pinB);

  /* this updates the counter                                */
  if (drB == HIGH) {   /* low->high on B? */

    if (drA == HIGH) { /* check pin A */
      Enc[1].pos++;  /* going clockwise: increment         */
    } else {
      Enc[1].pos--;  /* going counterclockwise: decrement  */
    }

  } else { /* must be high to low on B                       */

    if (drA == LOW) {  /* check pin A */
      Enc[1].pos++;  /* going clockwise: increment         */
    } else {
      Enc[1].pos--;  /* going counterclockwise: decrement  */
    }

  } /* end counter update                                    */

} /* end ISR pin B Encoder 1                                 */


/* Interrupt Service Routine: change on pin A for Encoder 2  */
void isrPinAEn2() {

  /* read pin B right away                                   */
  int drB = digitalRead(Enc[2].pinB);

  /* possibly wait before reading pin A, then read it        */
  debounce(Enc[2].del);
  int drA = digitalRead(Enc[2].pinA);

  /* this updates the counter                                */
  if (drA == HIGH) {   /* low->high on A? */

    if (drB == LOW) {  /* check pin B */
      Enc[2].pos++;  /* going clockwise: increment         */
    } else {
      Enc[2].pos--;  /* going counterclockwise: decrement  */
    }

  } else { /* must be high to low on A                       */

    if (drB == HIGH) { /* check pin B */
      Enc[2].pos++;  /* going clockwise: increment         */
    } else {
      Enc[2].pos--;  /* going counterclockwise: decrement  */
    }

  } /* end counter update                                    */

} /* end ISR pin A Encoder 2                                 */


/* Interrupt Service Routine: change on pin B for Encoder 2  */
void isrPinBEn2() {

  /* read pin A right away                                   */
  int drA = digitalRead(Enc[2].pinA);

  /* possibly wait before reading pin B, then read it        */
  debounce(Enc[2].del);
  int drB = digitalRead(Enc[2].pinB);

  /* this updates the counter                                */
  if (drB == HIGH) {   /* low->high on B? */

    if (drA == HIGH) { /* check pin A */
      Enc[2].pos++;  /* going clockwise: increment         */
    } else {
      Enc[2].pos--;  /* going counterclockwise: decrement  */
    }

  } else { /* must be high to low on B                       */

    if (drA == LOW) {  /* check pin A */
      Enc[2].pos++;  /* going clockwise: increment         */
    } else {
      Enc[2].pos--;  /* going counterclockwise: decrement  */
    }

  } /* end counter update                                    */

} /* end ISR pin B Encoder 2                                 */
