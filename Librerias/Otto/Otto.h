#ifndef Otto_h
#define Otto_h

#include <Servo.h>
#include <Oscillator.h>
#include <EEPROM.h>
//#include <EnableInterrupt.h> 

#include <US.h>
//#include <LedMatrix.h>
#include "MaxMatrix.h"
#include <BatReader.h>

#include "Otto_mouths.h"
#include "Otto_sounds.h"
#include "Otto_gestures.h"
#include "OttoSerialCommand.h"
//#include "SoftSerialCommand.h" //Library modified from: "SerialCommand.h" by Steven Cogswell http://awtfy.com

//-- Constants
#define FORWARD     1
#define BACKWARD    -1
#define LEFT        1
#define RIGHT       -1
#define SMALL       5
#define MEDIUM      15
#define BIG         30

#define PIN_YL 2 //servo[0] connect Servo Hip left to D2
#define PIN_YR 3 //servo[1] Connect Servo Hip right to D3
#define PIN_RL 4 //servo[2] Connect Servo Foot Left to D4
#define PIN_RR 5 //servo[3] COnnect Servo Foot Right to D5

#define PIN_Buzzer  17// [A3]
#define PIN_Trigger 8
#define PIN_Echo    9
#define PIN_NoiseSensor 16 //[A2] this is due [A6 is exclusive analogue] and not compatible with digital sound sensor

// Aditional Pins to compatibility with Otto
///define Bluetooth in SoftwareSerial 
#define BT_Rx   0 //6  
#define BT_Tx   1 //7
//define Max7219 pins 
#define PIN_DIN    10   //max 7219
#define PIN_CS     11
#define PIN_CLK    12
//define buttons Mode Action Otto
#define PIN_SecondButton 18
#define PIN_ThirdButton 19


class Otto
{
  public:

    //-- Otto initialization
    void init(int YL, int YR, int RL, int RR, bool load_calibration=true, int NoiseSensor=PIN_NoiseSensor, int Buzzer=PIN_Buzzer, int USTrigger=PIN_Trigger, int USEcho=PIN_Echo);

    //-- Attach & detach functions
    void attachServos();
    void detachServos();

    //-- Oscillator Trims
    void setTrims(int YL, int YR, int RL, int RR);
    void saveTrimsOnEEPROM();

    //-- Predetermined Motion Functions
    void _moveServos(int time, int  servo_target[]);
    void oscillateServos(int A[4], int O[4], int T, double phase_diff[4], float cycle);

    //-- HOME = Otto at rest position
    void home();
    bool getRestState();
    void setRestState(bool state);
    
    //-- Predetermined Motion Functions
    void jump(float steps=1, int T = 2000);

    void walk(float steps=4, int T=1000, int dir = FORWARD);
    void turn(float steps=4, int T=2000, int dir = LEFT);
    void bend (int steps=1, int T=1400, int dir=LEFT);
    void shakeLeg (int steps=1, int T = 2000, int dir=RIGHT);

    void updown(float steps=1, int T=1000, int h = 20);
    void swing(float steps=1, int T=1000, int h=20);
    void tiptoeSwing(float steps=1, int T=900, int h=20);
    void jitter(float steps=1, int T=500, int h=20);
    void ascendingTurn(float steps=1, int T=900, int h=20);

    void moonwalker(float steps=1, int T=900, int h=20, int dir=LEFT);
    void crusaito(float steps=1, int T=900, int h=20, int dir=FORWARD);
    void flapping(float steps=1, int T=1000, int h=20, int dir=FORWARD);
//    void move(int moveID,int time, int _moveSize);
    //-- Sensors functions
    float getDistance(); //US sensor
    int getNoise();      //Noise Sensor
	int getNoised();      //Noise Sensor digital

    //-- Battery
    double getBatteryLevel();
    double getBatteryVoltage();
    
    //-- Mouth & Animations
    void putMouth(unsigned long int mouth, bool predefined = true);
    void putAnimationMouth(unsigned long int anim, int index);
    void clearMouth();

    //-- Sounds
    void _tone (float noteFrequency, long noteDuration, int silentDuration);
//    void _playNote(float noteFrequency, long noteDuration);
    void bendTones (float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration);
    void sing(int songName);

    //-- Gestures
    void playGesture(int gesture);

 
  private:
    
    //LedMatrix ledmatrix;
    //LedMatrix ledmatrix=LedMatrix(PIN_DIN,PIN_CS,PIN_CLK, 1);  // init Max7219 LED Matrix, 1 module
	MaxMatrix ledmatrix=MaxMatrix(PIN_DIN,PIN_CS,PIN_CLK,1);
    BatReader battery;
    Oscillator servo[4];
    US us;

    int servo_pins[4];
    int servo_trim[4];
    int servo_position[4];

    int pinBuzzer;
    int pinNoiseSensor;
    
    unsigned long final_time;
    unsigned long partial_time;
    float increment[4];

    bool isOttoResting;

    unsigned long int getMouthShape(int number);
    unsigned long int getAnimShape(int anim, int index);
    void _execute(int A[4], int O[4], int T, double phase_diff[4], float steps);

};

#endif


