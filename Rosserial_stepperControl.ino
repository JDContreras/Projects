/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP, DIR as indicated
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
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
 * PROGRAMA
 *------------------------------------------------------------*/
void setup() {

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
current_time = micros();

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
  }//ifRvel 0
 /*
  else if (Lvel == 0 and Rvel == 0){
        digitalWrite(REnPin, LOW);
    digitalWrite(LEnPin, LOW);
  }
  

  if (Rvel == 0){
    digitalWrite(ledPin, LOW);
  }
  */
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
