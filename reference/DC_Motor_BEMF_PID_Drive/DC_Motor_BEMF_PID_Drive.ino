
//STAN2019, Mlock BEMF-Drive Proof of Concept
/*
   Author: stan

   Created on 10. Apr 2019
*/

//      Divisor: Setting: fmin:       fmax: 50% Wt
#include <avr/wdt.h>
#include "pid_regulator.h"
#include "TimerOne.h"
#define DIV1      0x01    //31250     8Mhz
#define DIV8      0x02    //3906.25   1Mhz
#define DIV32     0x03    //976.56    250Khz
#define DIV64     0x04    //488.28   <--DEFAULT
#define DIV128    0x05    //244.14
#define DIV256    0x06    //122.07
#define DIV1024   0x07    //30.51

#define MOTA       6       //Pin 6 is low speed PWM Out for Motor Control
#define MOTB       5       //Pin 5 is low speed PWM Out for Motor control

#define SensOpen  4       //Pin 4 is input for sensor position SENSE_AUF
#define SensSelf  2       //Pin 2 is input for sensor position SENSE_SELBST
#define SensSlide 8       //Pin 8 is input for sensor position SENSE_SCHLITTEN_SELBST
#define SensClose 12      //Pin 12 is input for sensor position SENSE_ZU_MIN

const int disable = 4; //Arduino pin 4 is disable - line
const int ADC0 = A0;  // Analog input pin 0

//----<Local Variables>----
byte inByte = -1;
byte DeBounce = 0;
float Wt = 50;             //Setpoint speed (270 is approx. 1 s)
int tick = 0;
double mean = 0;
stPIDParams MystPIDParams;
stPIDParams *pstPIDParams; //Define an empty struct pointer to type _stPIDParams
float PIDt = 0.005; //PID timer interrupt time t
int BEMF = 0; //Store BEMF voltage
float Out = 0;  //Output of Pid-Regulator
//----</Local Variables----

void setup() {
  Serial.begin(115200);
  pinMode(MOTA, OUTPUT);
  pinMode(MOTB, OUTPUT);
  pinMode(SensOpen, INPUT);
  pinMode(SensSelf, INPUT);
  pinMode(SensSlide, INPUT);
  pinMode(SensClose, INPUT);


  //TCCR0B = 0b00000001; // Set fucking prescaler for Timer 0 - pin 5 and 6 "AnalogWrite" functions to 1
  TCCR0B = 0b00000010; // Set fucking prescaler for Timer 0 - pin 5 and 6 "AnalogWrite" functions to 8
  //TCCR0B = 0b00000011; // Set fucking prescaler for Timer 0 - pin 5 and 6 "AnalogWrite" functions to 64

  analogWrite(MOTA, 0);
  analogWrite(MOTB, 0);
  //Serial.println("Initialised, waiting for command");



  pstPIDParams = &MystPIDParams;                  //Assigng struc pointer
  //ConfigPID(pstPIDParams, 0.7, 100, 0.0001, PIDt);        //struct, p, i, d, t.  for t = 0.0001
  ConfigPID(pstPIDParams, 0.5, 32, 0, PIDt);        //struct, p, i, d, t.  for t = 0.005 (200Hz) 2 20
  //ConfigPID(pstPIDParams, 2, 0.1, 0, PIDt);        //struct, p, i, d, t.  for t = 0.0025 (400Hz)

  Timer1.attachInterrupt(TmrIRQ); //Attach function to timer interrupt
  Timer1.initialize(1000000 * PIDt);    //Set timer interrupt time to PIDt periode definition and start timer interrupt
  wdt_enable(WDTO_8S);  //Enable WDT
}



void loop()
{
  wdt_reset(); //Reset WDT
  Timer1.stop(); //Stop timer interrupt


  // send data only when you receive data:
  if (Serial.available() > 0)
  {
    inByte = Serial.read();     //WASD game style control: read the incoming byte --> "w", "a" or "d". w = drive, a = speed down, d = speed up.
    //Serial.write(inByte);     //Debug

    if (inByte == 'w')    //Drive Command Received
    {
      //Serial.println("--> Travel Sense Open");
      Timer1.start();
      while (digitalRead(SensOpen) == true)         //1. Travel to SensOpen position
      {
        Drive(-Out);
      }
      pstPIDParams->Integral = 0; //Reset integral to 0
      Timer1.stop();
      Drive(0);

      //Serial.println("|| Pause 500ms");
      MyDelay(500);  //Wait 500ms                   //2. Wait 500ms

      //Serial.println("--> Travel Sense Self");
      Timer1.start();
      DeBounce = 0;
      while (DeBounce < 10)                          //3. Travel to SensSelf
      {
        Drive(Out);
        if (digitalRead(SensSelf) == false)
        {
          DeBounce++;
        }
        else
        {
          DeBounce = 0;
        }
        delayMicroseconds(250);
      }
      pstPIDParams->Integral = 0; //Reset integral to 0
      Timer1.stop();
      Drive(0);

      //Serial.println("( ) Wait for Sense Slide");
      while (digitalRead(SensSlide) == true)        //4. Wait for SensSlide to fall
      {
        //Serial.println(digitalRead(SensSelf));
        Drive(0);  //Do nothing
      }

      //Serial.println("--> Traveling Sense Close");
      Timer1.start();
      while (digitalRead(SensClose) == true)  //CLOSE
      {
        Drive(Out);
      }
      pstPIDParams->Integral = 0; //Reset integral to 0
      Timer1.stop();
      Drive(0);


    }
    
    if (inByte == 'd') //Wt up - Speed Up
    {
      Wt = Wt + 50;
      if (Wt > 250)
      {
        Wt = 250;
      }
    }
    if (inByte == 'a') //Wt down - Speed Down
    {
      Wt = Wt - 50;
      if (Wt < 0)
      {
        Wt = 0;
      }
    }
    while (Serial.read() >= 0) ; // flush the receive buffer.
  }


}





/* Function Drive
    Description: Sets PWMs for driving a DC-Motor with ti's DRV8871
    Input: speed, positive values drive left, negative values drive right,
    Output: none
    Side Effects: None observed.
*/
void Drive (int _speed)
{
  if (_speed < -250)  //Sanity Check under
  {
    _speed = -250;    //limit lower
  }
  if (_speed > 250)   //Sanity Check over
  {
    _speed = 250;     //limit upper
  }

  if (_speed < 0) //go left
  {
    digitalWrite(MOTB, 1);
    analogWrite(MOTA, 255 - abs(_speed));
  }
  else if (_speed > 0) //go right
  {
    digitalWrite(MOTA, 1);
    analogWrite(MOTB, 255 - abs(_speed));
  }
  else //Coast
  {
    analogWrite(MOTA, 0);
    analogWrite(MOTB, 0);
  }


}


void MyDelay(int _delay)
{
  for (int i = 0; i < _delay; i++)
  {
    delay(8);
  }
}

void TmrIRQ ()
{
  digitalWrite(MOTA, 0);
  digitalWrite(MOTB, 0);
  delayMicroseconds(200);
  BEMF = analogRead(ADC0);
  Out = PIDRegulator(Wt, BEMF, pstPIDParams);
}


/* Function ConfigPID
    Description: Configures all terms of the PID-Regulator.
    Input: Structure of PID parameters, proportional, intergral and deriviative term. Time intervall for loop time.
    Output: None
    Side Effects: None observed

*/
void ConfigPID (stPIDParams * pstPIDParams, float kp, float ki, float kd, float t)
{
  pstPIDParams->kp = kp;
  pstPIDParams->ki = ki;
  pstPIDParams->kd = kd;

  if (t > 0)  //Check if sane...
  {
    pstPIDParams->t = t;
  }
  else
  {
    pstPIDParams->t = 1; //Fallback: 1s
  }

}



/* Function PIDRegulator
    Description: Performs a software PID regulation loop.
    Input: w(t), y(t), PID parameters.
    Output: u(t)
    Side Effects: None observed.  Execute config PID before using this function
                      _____             _________
                     |     |           |         |
    w(t)--->O--e(t)->| PID |---u(t)--->| Strecke |---y(t)--->
            |        |_____|           |_________|   |
            |<----------------feedback---------------|

*/
float PIDRegulator (float wt, float yt, stPIDParams * pstPIDParams)
{
  float ut = 0;
  float et = 0;
  float deriviative = 0;

  et = wt - yt;                                                               //Get error from set value to output value


  if (et > 1 || et < 1)
  {
    //pstPIDParams->ut[2] = pstPIDParams->ut[1];                              //Shift filter array
    //pstPIDParams->ut[1] = pstPIDParams->ut[0];                              //Shift filter array

    pstPIDParams->Integral = pstPIDParams->Integral + et * pstPIDParams->t; //Add error to integral
    deriviative = (et - pstPIDParams->last_et) / pstPIDParams->t;           //Calculate deriviative term

    //                           Proportional                       Integral                                 Deriviative
    pstPIDParams->ut[0] = (pstPIDParams->kp * et) + (pstPIDParams->ki * pstPIDParams->Integral) + (pstPIDParams->kd * deriviative); //Calculate output

    pstPIDParams->last_et = et;                                             //Store last error for next deriviative calculations

  }

  //Serial.println("wt=" + String(wt) + " yt=" + String(yt) + " et=" + String(et) + " ut=" + String(pstPIDParams->ut[0]));  //Debug output
  //Serial.println(String(wt) + " " + String(yt) + " " + String(et) + " " + String(pstPIDParams->ut[0]));  //Debug output

  //return (pstPIDParams->ut[0]+pstPIDParams->ut[1]+pstPIDParams->ut[2])/3; //Return filtered output
  return pstPIDParams->ut[0];  //Return unfiltered output
}
