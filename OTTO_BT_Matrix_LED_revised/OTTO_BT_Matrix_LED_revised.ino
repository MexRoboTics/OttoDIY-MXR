//----------------------------------------------------------------
//-- OTTO-MXR basic firmware v2 based on Zowi basic firmware v2
//-- MexRoboTics. Released under a GPL licencse
//-- 11 May 2018
//--
//-- Originally Zowi Authors:  Anita de Prado: ana.deprado@bq.com
//--           Jose Alberca:   jose.alberca@bq.com
//--           Javier Isabel:  javier.isabel@bq.com
//--           Juan Gonzalez (obijuan): juan.gonzalez@bq.com
//--           Irene Sanz : irene.sanz@bq.com
//--
//-- Contributor OTTO-MXR: Rafael Adrian Garcia Garcia: mexrobotics.steam@gmail.com
//-----------------------------------------------------------------
//-- Experiment with all the features that Otto-MXR have and try to modify!
//-----------------------------------------------------------------
//-- Hardware conection Pins: Arduino Nano 
//   HC05   Tx  ----> Rx (0) Arduino 
//         Rx  ----> Tx (1) Arduino 
//-- This implicate that you should disconnect VCC Bluetooth when you upload the Sketch Arduino
//   SoundSensor         (A2 16) this is due (A6) is exclusive analogue and if you have a digital sensor you can't use these pin
//   Ultrasensor Echo    (D9)
//   Ultrasensor Trigger (D8)
//   MiniBuzzer          (A3 17)
//-- define Matrix led 8x8 Max7219 Pins
//   PIN_DIN             (D10)   //max 7219
//   PIN_CS              (D11)
//   PIN_CLK             (D12)
//   PIN_SecondButton    (D2)  // changes was made in this pins
//   PIN_ThirdButton     (D3)  // chages was made in this pins

//-- Otto Library
#include "Otto.h"
//-- Library to manage external interruptions (Second and Third buttons)
#include <EnableInterrupt.h> 
//-- Library to manage serial commands
#include "OttoSerialCommand.h"
OttoSerialCommand SCmd;  //The SerialCommand object

//SoftwareSerial BT = SoftwareSerial(BT_Rx,BT_Tx);//SoftwareSerial BT = SoftwareSerial(BT_Rx,BT_Tx); // Set Up Bluetooth connection on SoftwareSerial (Tx is D8, Rx is D7 in Arduino Side)  
//SoftSerialCommand SCmd(BT); //The SerialCommand object
// I try to use other  similar library to Software serial in order to re-define the Bluetooth Pins and don't disconnect VCC BT when upload the Sketch

//-- Create Otto Object :D
Otto Otto_MXR;  //This is Otto_MXR!!

MaxMatrix ledmatrix=MaxMatrix(PIN_DIN,PIN_CS,PIN_CLK,1); //10, 11, 12, 1 

//---------------------------------------------------------
//-- Configuration of pins where the servos are attached
/*
         --------------- 
        |               |
        |     O   O     |
        |               |
 YR ==> |               | <== YL
         --------------- 
            ||     ||
            ||     ||
            ||     ||
 RR ==>   -----   ------  <== RL
          -----   ------
//Declarations are in Otto.h
//  #define PIN_YL 2 //servo[0] 4
//  #define PIN_YR 3 //servo[1] 5
//  #define PIN_RL 4 //servo[2] 6
//  #define PIN_RR 5 //servo[3] 7
//---------------------------------------------------------
//---Otto_MXR Buttons original Second-> Pin 6 Third-> 7
//#define PIN_SecondButton 18
//#define PIN_ThirdButton 19
*/ 
///////////////////////////////////////////////////////////////////
//-- Global Variables -------------------------------------------//
///////////////////////////////////////////////////////////////////

const char programID[] = "Otto_MXR"; //Each program will have a ID
const char name_fac = '$'; //Factory name
const char name_fir = '#'; //First name

//-- Movement parameters
int T = 1000;              //Initial duration of movement
int moveId = 0;            //Number of movement
int modeId = 0;            //Number of mode
int moveSize = 15;         //Asociated with the height of some movements

//---------------------------------------------------------
//-- Otto_MXR has 5 modes:
//--    * MODE = 0: Otto_MXR is awaiting  
//--    * MODE = 1: Dancing mode!  
//--    * MODE = 2: Obstacle detector mode  
//--    * MODE = 3: Noise detector mode      *Some troubles with Digital Sensor
//--    * MODE = 4: Otto_MXRPAD or any Teleoperation mode (listening SerialPort). 
//--
volatile int MODE = 0; //State of Otto_MXR in the principal state machine. 
//--------------------------------------------------------- 

volatile bool buttonPushed = false;  //Variable to remember when a button has been pushed
volatile bool buttonAPushed = false; //Variable to remember when A button has been pushed
volatile bool buttonBPushed = false; //Variable to remember when B button has been pushed

unsigned long previousMillis=0;

int randomDance=0;
int randomSteps=0;

//-- RGB Led values
int led_R = 0;
int led_G = 0;
int led_B = 0;

bool obstacleDetected = false;


///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup(){

  //Serial communication initialization
  Serial.begin(115200); //init for Bluetooth HC-06 interface via Software Serial 
 
  //pinMode(PIN_SecondButton,INPUT_PULLUP); // So you need enter a 0V level to activate
  //pinMode(PIN_ThirdButton,INPUT_PULLUP);
         
  pinMode(PIN_SecondButton,INPUT); // So you need enter a 5V level to activate
  pinMode(PIN_ThirdButton,INPUT);
  
  //Set the servo pins
  Otto_MXR.init(PIN_YL,PIN_YR,PIN_RL,PIN_RR,false,PIN_NoiseSensor,PIN_Buzzer,PIN_Trigger,PIN_Echo);
  ledmatrix.init();
  ledmatrix.setIntensity(1);
  //Uncomment this to set the servo trims manually and save on EEPROM 
  //#define TRIM_RR 35
  //#define TRIM_RL 35
  //#define TRIM_YR 6
  //#define TRIM_YL 6
//  Otto_MXR.setTrims(TRIM_YL, TRIM_YR, TRIM_RL, TRIM_RR);
    
    //Otto_MXR.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.

  //Set a random seed
  randomSeed(125);//analogRead(A3) but I haven't analogue sensor

  //Interrumptions
  //enableInterrupt(PIN_SecondButton, secondButtonPushed, FALLING); // In case that in the previous pinMode you use only INPUT, change this by RISING
 // enableInterrupt(PIN_ThirdButton, thirdButtonPushed, FALLING);
  enableInterrupt(PIN_SecondButton, secondButtonPushed, RISING); // In case that in the previous pinMode you use INPUT_PULLUP, change this by FALLING
  enableInterrupt(PIN_ThirdButton, thirdButtonPushed, RISING);
         
  //Setup callbacks for SerialCommand commands 
  SCmd.addCommand("S", receiveStop);      //  sendAck & sendFinalAck
  SCmd.addCommand("L", receiveLED);       //  sendAck & sendFinalAck
  SCmd.addCommand("T", recieveBuzzer);    //  sendAck & sendFinalAck
  SCmd.addCommand("M", receiveMovement);  //  sendAck & sendFinalAck
  SCmd.addCommand("H", receiveGesture);   //  sendAck & sendFinalAck
  SCmd.addCommand("K", receiveSing);      //  sendAck & sendFinalAck
  SCmd.addCommand("C", receiveTrims);     //  sendAck & sendFinalAck
  SCmd.addCommand("G", receiveServo);     //  sendAck & sendFinalAck
  SCmd.addCommand("R", receiveName);      //  sendAck & sendFinalAck
  SCmd.addCommand("E", requestName);
  SCmd.addCommand("D", requestDistance);
  SCmd.addCommand("N", requestNoise);
  SCmd.addCommand("B", requestBattery);
  SCmd.addCommand("I", requestProgramId);
  SCmd.addCommand("J", requestMode);
  SCmd.addCommand("P", requestRGB);
  SCmd.addDefaultHandler(receiveStop);



  //Otto_MXR wake up!
  Otto_MXR.sing(S_connection);
  Otto_MXR.home();
  delay(50);
  Serial.println("Otto-MXR Started");

  //If Otto_MXR's name is '&' (factory name) means that is the first time this program is executed.
  //This first time, Otto_MXR mustn't do anything. Just born at the factory!
  //5 = EEPROM address that contains first name character
  if (EEPROM.read(5)==name_fac){ 

    EEPROM.put(5, name_fir); //From now, the name is '#'
    EEPROM.put(6, '\0'); 
    Otto_MXR.putMouth(culito);

    while(true){    
       delay(1000);
    }
  }  


  //Send Otto_MXR name, programID & battery level.
  requestName();
  delay(50);
  requestProgramId();
  delay(50);
  requestBattery();
  Serial.println("Sent Name");
  
  //Checking battery
  //OttoLowBatteryAlarm();


 // Animation Uuuuuh - A little moment of initial surprise
 //-----
  for(int i=0; i<2; i++){
      for (int i=0;i<8;i++){
        if(buttonPushed){break;}  
        Otto_MXR.putAnimationMouth(littleUuh,i);
        delay(150);
      }
  }
 //-----


  //Smile for a happy Otto_MXR :)
  if(!buttonPushed){ 
    Otto_MXR.putMouth(smile);
    Otto_MXR.sing(S_happy);
    delay(200);
  }


  //If Otto_MXR's name is '#' means that Otto_MXR hasn't been baptized
  //In this case, Otto_MXR does a longer greeting
  //5 = EEPROM address that contains first name character
  if (EEPROM.read(5)==name_fir){ 

    if(!buttonPushed){  
        Otto_MXR.jump(1,700);
        delay(200); 
    }

    if(!buttonPushed){  
        Otto_MXR.shakeLeg(1,T,1); 
    }  
    
    if(!buttonPushed){ 
        Otto_MXR.putMouth(smallSurprise);
        Otto_MXR.swing(2,800,20);  
        Otto_MXR.home();
    }  
  }


  if(!buttonPushed){ 
    Otto_MXR.putMouth(happyOpen);
  }

  previousMillis = millis();

}



///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {


  if (Serial.available()>0 && MODE!=4){

    MODE=4;
    Otto_MXR.putMouth(happyOpen);
    SCmd.readSerial();        /////////////
    //Disable Pin Interruptions
    disableInterrupt(PIN_SecondButton);
    disableInterrupt(PIN_ThirdButton);

    buttonPushed=false;
  }


  //First attemp to initial software
  if (buttonPushed){  

    Otto_MXR.home();

    delay(100); //Wait for all buttons 
    Otto_MXR.sing(S_buttonPushed);
    delay(200); //Wait for all buttons 

    if      ( buttonAPushed && !buttonBPushed){ MODE=1; Otto_MXR.sing(S_mode1);}
    else if (!buttonAPushed && buttonBPushed) { MODE=2; Otto_MXR.sing(S_mode2);}
    else if ( buttonAPushed && buttonBPushed) { MODE=3; Otto_MXR.sing(S_mode3);} //else

    Otto_MXR.putMouth(MODE);
 
    int showTime = 2000;
    while((showTime>0)){ //Wait to show the MODE number 
        
        showTime-=10;
        delay(10);
    }
     
    Otto_MXR.putMouth(happyOpen);

    buttonPushed=false;
    buttonAPushed=false;
    buttonBPushed=false;

  }else{

    switch (MODE) {

      //-- MODE 0 - Otto_MXR is awaiting
      //---------------------------------------------------------
      case 0:
      
        //Every 60 seconds in this mode, Otto_MXR falls asleep 
        if (millis()-previousMillis>=60000){
            OttoSleeping_withInterrupts(); //ZZzzzzz...
            previousMillis=millis();         
        }

        break;
      

      //-- MODE 1 - Dance Mode!
      //---------------------------------------------------------
      case 1:
        
        randomDance=random(5,21); //5,20
        if((randomDance>14)&&(randomDance<19)){
            randomSteps=1;
            T=1600;
        }
        else{
            randomSteps=random(3,6); //3,5
            T=1000;
        }
        
        Otto_MXR.putMouth(random(10,21));

        for (int i=0;i<randomSteps;i++){
            move(randomDance);
            if(buttonPushed){break;}
        }
        break;


      //-- MODE 2 - Obstacle detector mode
      //---------------------------------------------------------
      case 2:

        if(obstacleDetected){

            if(!buttonPushed){
              Otto_MXR.putMouth(bigSurprise);
              Otto_MXR.sing(S_surprise);
              Otto_MXR.jump(5, 500);
            }  

            if(!buttonPushed){
              Otto_MXR.putMouth(confused);
              Otto_MXR.sing(S_cuddly);
            }  

            //Otto_MXR takes two steps back
            for(int i=0;i<3;i++){ 
              if(buttonPushed){break;}
              Otto_MXR.walk(1,1300,-1);
            }

            delay(100);
            obstacleDetector();
            delay(100);


           //If there are no obstacles and no button is pressed, Otto_MXR shows a smile
           if((obstacleDetected==true)||(buttonPushed==true)){break;}            
           else{
              Otto_MXR.putMouth(smile);
              delay(50);
              obstacleDetector();
           } 
            
           
           //If there are no obstacles and no button is pressed, Otto_MXR shows turns left
           for(int i=0; i<3; i++){
              if((obstacleDetected==true)||(buttonPushed==true)){break;}            
              else{ 
                  Otto_MXR.turn(1,1000,1); 
                  obstacleDetector();
              } 
           }
            
            //If there are no obstacles and no button is pressed, Otto_MXR is happy
            if((obstacleDetected==true)||(buttonPushed==true)){break;}           
            else{
                Otto_MXR.home();
                Otto_MXR.putMouth(happyOpen);
                Otto_MXR.sing(S_happy_short);
                delay(200);
            }     
        

        }else{

            Otto_MXR.walk(1,1000,1); //Otto_MXR walk straight
            obstacleDetector();
        }   

        break;


      //-- MODE 3 - Noise detector mode
      //---------------------------------------------------------  
      case 3:

        delay(1500); //Wait for possible noise of the servos while get home

        if (Otto_MXR.getNoised()){ //740 // this is due my sensor is digital, if your sensor is analoge use only Otto_MXR.getNoise()>= 650
          
          delay(100);  //Wait for the possible 'lag' of the button interruptions. 
                      //Sometimes, the noise sensor detect the button before the interruption takes efect 

          if(!buttonPushed){

            Otto_MXR.putMouth(bigSurprise);
            Otto_MXR.sing(S_OhOoh);

            if(buttonPushed)
            {
              break;
            }
            Otto_MXR.putMouth(random(10,21));
            randomDance=random(5,21);
            move(randomDance);
            Otto_MXR.home();
            delay(1000); //Wait for possible noise of the servos while get home
          }
          
          if(!buttonPushed)
          {
            Otto_MXR.putMouth(happyOpen);
          }

        }
        break;
        

      //-- MODE 4 - Otto_MXRPAD or any Teleoperation mode (listening SerialPort) 
      //---------------------------------------------------------
      case 4:

        SCmd.readSerial();
        
        //If Otto_MXR is moving yet
        if (Otto_MXR.getRestState()==false){  
          move(moveId);
        }
      
        break;      


      default:
          MODE=4;//4
          break;
    }

  } 

}  



///////////////////////////////////////////////////////////////////
//-- Functions --------------------------------------------------//
///////////////////////////////////////////////////////////////////

//-- Function executed when second button is pushed
void secondButtonPushed(){ 

    buttonAPushed=true;

    if(!buttonPushed){
        buttonPushed=true;
        Otto_MXR.putMouth(smallSurprise);
    }    
}

//-- Function executed when third button is pushed
void thirdButtonPushed(){ 

    buttonBPushed=true;

    if(!buttonPushed){
        buttonPushed=true;
        Otto_MXR.putMouth(smallSurprise);
    }
}


//-- Function to read distance sensor & to actualize obstacleDetected variable
void obstacleDetector(){

   int distance = Otto_MXR.getDistance();

        if(distance<15){
          obstacleDetected = true;
        }else{
          obstacleDetected = false;
        }
}


//-- Function to receive Stop command.
void receiveStop(){

    sendAck();
    Otto_MXR.home();
    sendFinalAck();

}


//-- Function to receive LED commands
void receiveLED(){  

    //sendAck & stop if necessary
    sendAck();
    Otto_MXR.home();

    //Examples of receiveLED Bluetooth commands
    //L 000000001000010100100011000000000
    //L 001111111111111111111111111111111 (todos los LED encendidos)
    unsigned long int matrix;
    char *arg;
    char *endstr;
    arg=SCmd.next();
    //Serial.println (arg);
    if (arg != NULL) {
      matrix=strtoul(arg,&endstr,2);    // Converts a char string to unsigned long integer
      Otto_MXR.putMouth(matrix,false);
    }else{
      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();
    }

    sendFinalAck();

}


//-- Function to receive buzzer commands
void recieveBuzzer(){
  
    //sendAck & stop if necessary
    sendAck();
    Otto_MXR.home(); 

    bool error = false; 
    int frec;
    int duration; 
    char *arg; 
    
    arg = SCmd.next(); 
    if (arg != NULL) { frec=atoi(arg); }    // Converts a char string to an integer   
    else {error=true;}
    
    arg = SCmd.next(); 
    if (arg != NULL) { duration=atoi(arg); } // Converts a char string to an integer  
    else {error=true;}

    if(error==true){

      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();

    }else{ 

      Otto_MXR._tone(frec, duration, 1);   
    }

    sendFinalAck();

}


//-- Function to receive TRims commands
void receiveTrims(){  

    //sendAck & stop if necessary
    sendAck();
    Otto_MXR.home(); 

    int trim_YL,trim_YR,trim_RL,trim_RR;

    //Definition of Servo Bluetooth command
    //C trim_YL trim_YR trim_RL trim_RR
    //Examples of receiveTrims Bluetooth commands
    //C 20 0 -8 3
    bool error = false;
    char *arg;
    arg=SCmd.next();
    if (arg != NULL) { trim_YL=atoi(arg); }    // Converts a char string to an integer   
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { trim_YR=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { trim_RL=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { trim_RR=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}
    
    if(error==true){

      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();

    }else{ //Save it on EEPROM
      Otto_MXR.setTrims(trim_YL, trim_YR, trim_RL, trim_RR);
      Otto_MXR.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.
    } 

    sendFinalAck();

}


//-- Function to receive Servo commands
void receiveServo(){  

    sendAck(); 
    moveId = 30;

    //Definition of Servo Bluetooth command
    //G  servo_YL servo_YR servo_RL servo_RR 
    //Example of receiveServo Bluetooth commands
    //G 90 85 96 78 
    bool error = false;
    char *arg;
    int servo_YL,servo_YR,servo_RL,servo_RR;

    arg=SCmd.next();
    if (arg != NULL) { servo_YL=atoi(arg); }    // Converts a char string to an integer   
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { servo_YR=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { servo_RL=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { servo_RR=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}
    
    if(error==true){

      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();

    }else{ //Update Servo:

      int servoPos[4]={servo_YL, servo_YR, servo_RL, servo_RR}; 
      Otto_MXR._moveServos(200, servoPos);   //Move 200ms
      
    }

    sendFinalAck();

}


//-- Function to receive movement commands
void receiveMovement(){

    sendAck();
    Serial.print("Move Command: ");
    if (Otto_MXR.getRestState()==true){
        Otto_MXR.setRestState(false);
    }

    //Definition of Movement Bluetooth commands
    //M  MoveID  T   MoveSize  
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {moveId=atoi(arg);Serial.println(moveId); Serial.print(" ");}
    else{
      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();
      moveId=0; //stop
    }
    
    arg = SCmd.next(); 
    if (arg != NULL) {T=atoi(arg);Serial.println(T); Serial.print(" ");}
    else{
      T=1000;
    }

    arg = SCmd.next(); 
    if (arg != NULL) {moveSize=atoi(arg);Serial.println(moveSize); Serial.print(" ");}
    else{
      moveSize =15;
    }
}


//-- Function to execute the right movement according the movement command received.
void move(int moveId){

  bool manualMode = false;

  switch (moveId) {
    case 0:
      Otto_MXR.home();
      break;
    case 1: //M 1 1000 
      Otto_MXR.walk(1,T,1);
      break;
    case 2: //M 2 1000 
      Otto_MXR.walk(1,T,-1);
      break;
    case 3: //M 3 1000 
      Otto_MXR.turn(1,T,1);
      break;
    case 4: //M 4 1000 
      Otto_MXR.turn(1,T,-1);
      break;
    case 5: //M 5 1000 30 
      Otto_MXR.updown(1,T,moveSize);
      break;
    case 6: //M 6 1000 30
      Otto_MXR.moonwalker(1,T,moveSize,1);
      break;
    case 7: //M 7 1000 30
      Otto_MXR.moonwalker(1,T,moveSize,-1);
      break;
    case 8: //M 8 1000 30
      Otto_MXR.swing(1,T,moveSize);
      break;
    case 9: //M 9 1000 30 
      Otto_MXR.crusaito(1,T,moveSize,1);
      break;
    case 10: //M 10 1000 30 
      Otto_MXR.crusaito(1,T,moveSize,-1);
      break;
    case 11: //M 11 1000 
      Otto_MXR.jump(1,T);
      break;
    case 12: //M 12 1000 30 
      Otto_MXR.flapping(1,T,moveSize,1);
      break;
    case 13: //M 13 1000 30
      Otto_MXR.flapping(1,T,moveSize,-1);
      break;
    case 14: //M 14 1000 20
      Otto_MXR.tiptoeSwing(1,T,moveSize);
      break;
    case 15: //M 15 500 
      Otto_MXR.bend(1,T,1);
      break;
    case 16: //M 16 500 
      Otto_MXR.bend(1,T,-1);
      break;
    case 17: //M 17 500 
      Otto_MXR.shakeLeg(1,T,1);
      break;
    case 18: //M 18 500 
      Otto_MXR.shakeLeg(1,T,-1);
      break;
    case 19: //M 19 500 20
      Otto_MXR.jitter(1,T,moveSize);
      break;
    case 20: //M 20 500 15
      Otto_MXR.ascendingTurn(1,T,moveSize);
      break;
    default:
        manualMode = true;
      break;
  }

  if(!manualMode){
    sendFinalAck();
  }
       
}


//-- Function to receive gesture commands
void receiveGesture(){

    //sendAck & stop if necessary
    sendAck();
    Otto_MXR.home(); 

    //Definition of Gesture Bluetooth commands
    //H  GestureID  
    int gesture = 0;
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {gesture=atoi(arg);}
    else 
    {
      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();
    }

    switch (gesture) {
      case 1: //H 1 
        Otto_MXR.playGesture(OttoHappy);
        break;
      case 2: //H 2 
        Otto_MXR.playGesture(OttoSuperHappy);
        break;
      case 3: //H 3 
        Otto_MXR.playGesture(OttoSad);
        break;
      case 4: //H 4 
        Otto_MXR.playGesture(OttoSleeping);
        break;
      case 5: //H 5  
        Otto_MXR.playGesture(OttoFart);
        break;
      case 6: //H 6 
        Otto_MXR.playGesture(OttoConfused);
        break;
      case 7: //H 7 
        Otto_MXR.playGesture(OttoLove);
        break;
      case 8: //H 8 
        Otto_MXR.playGesture(OttoAngry);
        break;
      case 9: //H 9  
        Otto_MXR.playGesture(OttoFretful);
        break;
      case 10: //H 10
        Otto_MXR.playGesture(OttoMagic);
        break;  
      case 11: //H 11
        Otto_MXR.playGesture(OttoWave);
        break;   
      case 12: //H 12
        Otto_MXR.playGesture(OttoVictory);
        break; 
      case 13: //H 13
        Otto_MXR.playGesture(OttoFail);
        break;         
      default:
        break;
    }

    sendFinalAck();
}

//-- Function to receive sing commands
void receiveSing(){

    //sendAck & stop if necessary
    sendAck();
    Otto_MXR.home(); 

    //Definition of Sing Bluetooth commands
    //K  SingID    
    int sing = 0;
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {sing=atoi(arg);}
    else 
    {
      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();
    }

    switch (sing) {
      case 1: //K 1 
        Otto_MXR.sing(S_connection);
        break;
      case 2: //K 2 
        Otto_MXR.sing(S_disconnection);
        break;
      case 3: //K 3 
        Otto_MXR.sing(S_surprise);
        break;
      case 4: //K 4 
        Otto_MXR.sing(S_OhOoh);
        break;
      case 5: //K 5  
        Otto_MXR.sing(S_OhOoh2);
        break;
      case 6: //K 6 
        Otto_MXR.sing(S_cuddly);
        break;
      case 7: //K 7 
        Otto_MXR.sing(S_sleeping);
        break;
      case 8: //K 8 
        Otto_MXR.sing(S_happy);
        break;
      case 9: //K 9  
        Otto_MXR.sing(S_superHappy);
        break;
      case 10: //K 10
        Otto_MXR.sing(S_happy_short);
        break;  
      case 11: //K 11
        Otto_MXR.sing(S_sad);
        break;   
      case 12: //K 12
        Otto_MXR.sing(S_confused);
        break; 
      case 13: //K 13
        Otto_MXR.sing(S_fart1);
        break;
      case 14: //K 14
        Otto_MXR.sing(S_fart2);
        break;
      case 15: //K 15
        Otto_MXR.sing(S_fart3);
        break;    
      case 16: //K 16
        Otto_MXR.sing(S_mode1);
        break; 
      case 17: //K 17
        Otto_MXR.sing(S_mode2);
        break; 
      case 18: //K 18
        Otto_MXR.sing(S_mode3);
        break;   
      case 19: //K 19
        Otto_MXR.sing(S_buttonPushed);
        break;                      
      default:
        break;
    }

    sendFinalAck();
}


//-- Function to receive Name command
void receiveName(){

    //sendAck & stop if necessary
    sendAck();
    Otto_MXR.home(); 

    char newOttoName[11] = "";  //Variable to store data read from Serial.
    int eeAddress = 5;          //Location we want the data to be in EEPROM.
    char *arg; 
    arg = SCmd.next(); 
    
    if (arg != NULL) {

      //Complete newOttoName char string
      int k = 0;
      while((*arg) && (k<11)){ 
          newOttoName[k]=*arg++;
          k++;
      }
      
      EEPROM.put(eeAddress, newOttoName); 
    }
    else 
    {
      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();
    }

    sendFinalAck();

}


//-- Function to send Otto_MXR's name
void requestName(){

    Otto_MXR.home(); //stop if necessary

    char actualOttoName[11]= "";  //Variable to store data read from EEPROM.
    int eeAddress = 5;            //EEPROM address to start reading from

    //Get the float data from the EEPROM at position 'eeAddress'
    EEPROM.get(eeAddress, actualOttoName);

    Serial.print(F("&&"));
    Serial.print(F("E "));
    Serial.print(actualOttoName);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send ultrasonic sensor measure (distance)
void requestDistance(){

    Otto_MXR.home();  //stop if necessary  

    int distance = Otto_MXR.getDistance();
    Serial.print(F("&&"));
    Serial.print(F("D "));
    Serial.print(distance);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send noise sensor measure
void requestNoise(){

    Otto_MXR.home();  //stop if necessary

    int microphone= Otto_MXR.getNoised(); //analogRead(PIN_NoiseSensor);
    Serial.print(F("&&"));
    Serial.print(F("N "));
    Serial.print(microphone);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send battery voltage percent
void requestBattery(){

    Otto_MXR.home();  //stop if necessary

    //The first read of the batery is often a wrong reading, so we will discard this value. 
    double batteryLevel = Otto_MXR.getBatteryLevel();

    Serial.print(F("&&"));
    Serial.print(F("B "));
    Serial.print(batteryLevel);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send program ID
void requestProgramId(){

    Otto_MXR.home();   //stop if necessary

    Serial.print(F("&&"));
    Serial.print(F("I "));
    Serial.print(programID);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send Ack comand (A)
void sendAck(){

  delay(30);

  Serial.print(F("&&"));
  Serial.print(F("A"));
  Serial.println(F("%%"));
  Serial.flush();
}


//-- Function to send final Ack comand (F)
void sendFinalAck(){

  delay(30);

  Serial.print(F("&&"));
  Serial.print(F("F"));
  Serial.println(F("%%"));
  Serial.flush();
}

//-- Function to receive mode selection.
void requestMode(){

    sendAck();
    Otto_MXR.home();
    //Definition of Mode Bluetooth commands
    //J ModeID    
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) 
    {
      modeId=atoi(arg);
      Otto_MXR.putMouth(heart);
      }
    else{
      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();
      modeId=0; //stop
    }
switch (modeId) {
      case 0: //
        MODE = 0;
        break;
      case 1: //
        MODE = 1;
        Otto_MXR.sing(S_mode1);
        Otto_MXR.putMouth(one);
        delay(1000);
    delay(200);
        break;
        case 2: //
        MODE = 2;
        Otto_MXR.sing(S_mode2);
        Otto_MXR.putMouth(two);
        delay(1000);
        break;
        case 3: //
        MODE = 3;
        Otto_MXR.sing(S_mode3);
        Otto_MXR.putMouth(three);
        delay(1000);        
        break;
        case 4: //
        Otto_MXR.sing(S_mode1);
        Otto_MXR.putMouth(four);
        delay(1000);       
        MODE = 4;
        break;
     default:
        MODE = 0;
        break;
   }
sendFinalAck();
 }
//-- Function to receive RGB colours.
void requestRGB(){

    sendAck();
    Otto_MXR.home();
    //Definition of Mode Bluetooth commands
    //J ModeID    
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) 
    {
      led_R=atoi(arg);
      }
      else{
      Otto_MXR.putMouth(xMouth);
      delay(2000);
      Otto_MXR.clearMouth();
      led_R=0; //stop
    }
    
    arg = SCmd.next(); 
    if (arg != NULL) {led_G=atoi(arg);}
    else{
      led_G=0;
    }

    arg = SCmd.next(); 
    if (arg != NULL) {led_B=atoi(arg);}
    else{
      led_B =0;
    }
    Otto_MXR.putMouth(okMouth);
      delay(2000);
      Otto_MXR.clearMouth();
}

//-- Functions with animatics
//--------------------------------------------------------

void OttoLowBatteryAlarm(){

    double batteryLevel = Otto_MXR.getBatteryLevel();

    if(batteryLevel<45){
        
      while(!buttonPushed){

          Otto_MXR.putMouth(thunder);
          Otto_MXR.bendTones (880, 2000, 1.04, 8, 3);  //A5 = 880
          
          delay(30);

          Otto_MXR.bendTones (2000, 880, 1.02, 8, 3);  //A5 = 880
          Otto_MXR.clearMouth();
          delay(500);
 
      } 
    }

}

void OttoSleeping_withInterrupts(){

  int bedPos_0[4]={100, 80, 60, 120}; //{100, 80, 40, 140}

  if(!buttonPushed){
    Otto_MXR._moveServos(700, bedPos_0);  //800  
  }

  for(int i=0; i<4;i++){

    if(buttonPushed){break;}
      Otto_MXR.putAnimationMouth(dreamMouth,0);
      Otto_MXR.bendTones (100, 200, 1.04, 10, 10);
    
    if(buttonPushed){break;}
      Otto_MXR.putAnimationMouth(dreamMouth,1);
      Otto_MXR.bendTones (200, 300, 1.04, 10, 10);  

    if(buttonPushed){break;}
      Otto_MXR.putAnimationMouth(dreamMouth,2);
      Otto_MXR.bendTones (300, 500, 1.04, 10, 10);   

    delay(500);
    
    if(buttonPushed){break;}
      Otto_MXR.putAnimationMouth(dreamMouth,1);
      Otto_MXR.bendTones (400, 250, 1.04, 10, 1); 

    if(buttonPushed){break;}
      Otto_MXR.putAnimationMouth(dreamMouth,0);
      Otto_MXR.bendTones (250, 100, 1.04, 10, 1); 
    
    delay(500);
  } 

  if(!buttonPushed){
    Otto_MXR.putMouth(lineMouth);
    Otto_MXR.sing(S_cuddly);
  }

  Otto_MXR.home();
  if(!buttonPushed){Otto_MXR.putMouth(happyOpen);}  
}
