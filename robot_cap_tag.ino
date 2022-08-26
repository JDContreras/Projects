#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

#include <Arduino.h>
#include "BasicStepperDriver.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>

/* -----------------------------------------------------------
 * VARIABLES
 *------------------------------------------------------------*/
double Lvel = 0;
double Rvel = 0;
int Rcount = 0;
int Lcount = 0;


unsigned long current_time = 0;
unsigned long next_Ltime = micros();
unsigned long next_Rtime = micros();
unsigned long spinTime = 25000; //tiempo en el que se actualiza los pub & sub
unsigned long NextSpinTime = micros() + spinTime; //tiempo en el que se actualiza los pub & sub
int Lpulse_low_time = 50*100 - 400;
int Rpulse_low_time = 50*100 - 400; //tiempo de pulse en low, lo minimo es 400 para 180 rpm

/* -----------------------------------------------------------
 * MOTORES
 *------------------------------------------------------------*/
#define pulse_high_time  = 400 //400miliseconds tiempo de pulso -- es constante y varia es el tiempo de low

// configuracion de cables motor izquierdo
#define LDirPin 8 //direccion
#define LPulsePin 9 //púlsos
#define LEnPin 10 //enable

// configuracion de cables motor derecho
#define RDirPin 5 //direccion
#define RPulsePin 6 //púlsos
#define REnPin 7 //enable

#define ledPin 13 //led

/* -----------------------------------------------------------
 * CALLBACKS
 *------------------------------------------------------------*/

void LCallback( const std_msgs::Float32& vel){
  Lvel = vel.data;
}

void RCallback( const std_msgs::Float32& vel){
  Rvel = vel.data;
}

/* -----------------------------------------------------------
 * ROS
 *------------------------------------------------------------*/
ros::NodeHandle nh;
//suscripciones  -- lee la velocidad poblicada por el Hardware interface Node
ros::Subscriber<std_msgs::Float32> sub1("left_vel", LCallback);
ros::Subscriber<std_msgs::Float32> sub2("right_vel", RCallback);
//publicaciones  -- envia el conteo de pulsos a el Hardware interface Node
std_msgs::Int64 Lcount_msg;
ros::Publisher LCounter("left_count", &Lcount_msg);
std_msgs::Int64 Rcount_msg;
ros::Publisher RCounter("right_count", &Rcount_msg);

/* -----------------------------------------------------------
 * HUSKYLENS
 *------------------------------------------------------------*/

HUSKYLENS huskylens;

// Definition of variables
//HUSKYLENS green line >> SDA; blue line >> SCL

int ID0 = 0; //not learned results. Grey result on HUSKYLENS screen
int ID1 = 1; //first learned results. colored result on HUSKYLENS screen
int ID2 = 2; //second learned results. colored result on HUSKYLENS screen

int widthLevel = 50;

int xLeft = 160-40;
int xRight = 160+40;

bool isTurning = false;
bool isTurningLeft = true;

bool isInside(int value, int min, int max){
    return (value >= min && value <= max);
}

/* -----------------------------------------------------------
 * PROGRAMA
 *------------------------------------------------------------*/
 
void setup() {
  
  // Setup code here, to run once:
      
    Serial.begin(115200);
    Wire.begin();
    while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }

    huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
    

  pinMode(LDirPin, OUTPUT);
  pinMode(LPulsePin, OUTPUT);
  pinMode(LEnPin, OUTPUT);
  pinMode(RDirPin, OUTPUT);
  pinMode(RPulsePin, OUTPUT);
  pinMode(REnPin, OUTPUT);

  pinMode(ledPin, OUTPUT);
  
  nh.initNode();
  
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  
  nh.advertise(LCounter);
  nh.advertise(RCounter);
    
}

void loop() {
  // Main code here:
  current_time = micros();
  
  if (huskylens.requestLearned())   
  {       
     HUSKYLENSResult result = huskylens.read();

        if (result.width < widthLevel){
            widthLevel = 65;
            if (isInside(result.xCenter, 0, xLeft)){
                if (isTurningLeft){
                    if (!isTurning){
                        digitalWrite(LDirPin, HIGH);
                    }
                }
                else{
                    if (isTurning){
                        isTurning = false;
                        isTurningLeft = !isTurningLeft;
                    }
                    digitalWrite(LDirPin, HIGH);
                }
            }
            else if (isInside(result.xCenter, xLeft, xRight)){
                if (isTurning){
                    isTurning = false;
                    isTurningLeft = !isTurningLeft;
                }
                digitalWrite(LDirPin, HIGH);
                digitalWrite(RDirPin, HIGH);
            }
            else if (isInside(result.xCenter, xRight, 320)){
                if (isTurningLeft){
                    if (isTurning){
                        isTurning = false;
                        isTurningLeft = !isTurningLeft;
                    }
                    digitalWrite(RDirPin, HIGH);
                }
                else{
                    if (!isTurning){
                       digitalWrite(RDirPin, HIGH);
                    }
                }
            }

            //ENVIO DE PULSOS
            current_time = micros();
            if (current_time >= next_Ltime){
              digitalWrite(LPulsePin, HIGH);
              delayMicroseconds(400);
              next_Ltime = micros() + Lpulse_low_time;
              if(LDirPin){Lcount +=1;} else {Lcount -= 1;}
            }
            else if( current_time < next_Ltime) {
              digitalWrite(LPulsePin, LOW);
            }
            //ENVIO DE PULSOS
            if (current_time >= next_Rtime){
              digitalWrite(RPulsePin, HIGH);
              digitalWrite(ledPin, HIGH);
              delayMicroseconds(400);
              if(RDirPin){Rcount +=1;} else {Rcount -= 1;}
              next_Rtime = micros() + Rpulse_low_time;
            }
            else if( current_time < next_Rtime) {
              digitalWrite(RPulsePin, LOW);
              digitalWrite(ledPin, LOW);
            }
        }
        else
        {
            widthLevel = 55;
            isTurning = true;
            if (isTurningLeft){
                if (Rvel != 0){
    
                        //digitalWrite(REnPin, LOW); //verificar polaridad
                        //digitalWrite(LEnPin, LOW);
                        //convertir de rad/s a los tiempo del pulso
                        //cada paso (400/giro) tiene 0.9 grados = 0.015708 rad
                        //solo es una regla de 3, si debe recorrer X rad en 1 segundo, 0.015708 radianes en cuantos segundos?
                        Rpulse_low_time = 1000000*(0.015708/abs(Rvel))-400; //debe quedar en micros
                        //Rpulse_low_time = 4000;
                        //CAMBIO DE DIRECCION
                        if (Rvel > 0){
                          digitalWrite(RDirPin, LOW);
                        }
                        else if (Rvel < 0){
                          digitalWrite(RDirPin, HIGH);
                        }
                    
                        //ENVIO DE PULSOS
                        if (current_time >= next_Rtime){
                          digitalWrite(RPulsePin, HIGH);
                          digitalWrite(ledPin, HIGH);
                          delayMicroseconds(400);
                          if(RDirPin){Rcount +=1;} else {Rcount -= 1;}
                          next_Rtime = micros() + Rpulse_low_time;
                        }
                        else if( current_time < next_Rtime) {
                          digitalWrite(RPulsePin, LOW);
                          digitalWrite(ledPin, LOW);
                        }
                      }
            }
            else{
              if (Lvel != 0){
                  
                  //convertir de rad/s a los tiempo del pulso
                  //cada paso (400/giro) tiene 0.9 grados = 0.015708 rad
                  //solo es una regla de 3, si debe recorrer X rad en 1 segundo, 0.015708 radianes en cuantos segundos?
                  Lpulse_low_time = 1000000*(0.015708/abs(Lvel))-400; //deque quedar en micros
              
                  //CAMBIO DE DIRECCION
                  if (Lvel > 0){
                    digitalWrite(LDirPin, HIGH);
                  }
                  else if (Lvel < 0){
                    digitalWrite(LDirPin,LOW);
                  }
              
                  //ENVIO DE PULSOS
                  current_time = micros();
                  if (current_time >= next_Ltime){
                    digitalWrite(LPulsePin, HIGH);
                    delayMicroseconds(400);
                    next_Ltime = micros() + Lpulse_low_time;
                    if(LDirPin){Lcount +=1;} else {Lcount -= 1;}
                  }
                  else if( current_time < next_Ltime) {
                    digitalWrite(LPulsePin, LOW);
                  }
                }//if Lvel 0 
            }
        }
  }
  
  else
    {
        Serial.println("Fail to request objects from Huskylens!");
    }

  current_time = micros();
  nh.spinOnce();
  if (current_time > NextSpinTime) {
    Lcount_msg.data = Lcount;
    LCounter.publish( &Lcount_msg );
    Rcount_msg.data = Rcount;
    RCounter.publish( &Rcount_msg );
    NextSpinTime += micros()+ spinTime;
  }
}
