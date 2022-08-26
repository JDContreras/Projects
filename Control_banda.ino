
/* -----------------------------------------------------------
   librerias
  ------------------------------------------------------------*/


#include <Arduino.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

/* -----------------------------------------------------------
   Declarar Bluetooth
  ------------------------------------------------------------*/


// Check if Bluetooth configs are enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

/* -----------------------------------------------------------
   VARIABLES
  ------------------------------------------------------------*/

double vel = 60;
double v0 = 30;
int count = 0;
boolean ho = LOW;
int pusherpos = 0;
int nextPos = 0;


unsigned long current_time = 0;
unsigned long next_time = micros();
int pulse_low_time = 50 * 100 - 400; //tiempo de pulse en low, lo minimo es 400 para 180 rpm

// Handle received and sent messages
String message = "";
char incomingChar;
boolean inputComplete = false;

//variables programables por serial
int D = 5;
int A = 50;

int limit = 465;
int jump = int(limit / D);
int checkPos = limit;
/* -----------------------------------------------------------
   MOTORES
  ------------------------------------------------------------*/
#define pulse_high_time  = 400 //400miliseconds tiempo de pulso -- es constante y varia es el tiempo de low

// configuracion de cables motor banda
#define BDir 16 //direccion
#define BPul 32 //púlsos
#define BEn 17 //enable

// configuracion de cables motor pusher
#define PDir 25 //direccion
#define PPul 26 //púlsos
#define PEn 33 //enable

//parametros motor banda
int Bmsteps = 8;
int BPer = 140;

//parametros motor pusher
int Pmsteps = 8;
int PPer = 40;

// Parametros para el control del servo

int PinServo = 27;
Servo servo;
int top = 25;
int down = 180;

/* -----------------------------------------------------------
   Sensores
  ------------------------------------------------------------*/

int EndStop = 14;

/* -----------------------------------------------------------
   PROGRAMA
  ------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  SerialBT.begin("FEEDER");
  pinMode(BDir, OUTPUT);
  pinMode(BPul, OUTPUT);
  pinMode(BEn, OUTPUT);

  pinMode(PDir, OUTPUT);
  pinMode(PPul, OUTPUT);
  pinMode(PEn, OUTPUT);
  pinMode(EndStop, INPUT);
  servo.attach(PinServo);
  //Serial.begin(57600);
  goUp();
  Serial.println("Ok");

}

void loop() {
   //Serial.println(digitalRead(EndStop));
  serialEvent();
  if (inputComplete) {
    comandos(message);
    // clear the command:
    message = "";
    inputComplete = false;
    //Serial.println("Ok1");
  }



}

void serialEvent() {
  while (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar == '\n') {
      inputComplete = true;
      break;
    }
    message += incomingChar;
  }
}

void comandos(String val) { // d8 home h feed
  char valchar = val.charAt(0); // varchar = "f"
  String val2 = val.substring(1); //val2 = "fff"
  int val2_int = val2.toInt();  // val2_int = 4325435
  switch (valchar) {
    case 'h':
      Serial.println("homming");
      homing(PPul, PDir, Pmsteps);
      Serial.println("homming_OK");
      ho = HIGH;
      pusherpos = 0;
      Serial.println(limit - jump);
      pusherpos = PosControl(limit - jump, pusherpos);
      nextPos = pusherpos - jump;
      break;
    case 'f':
      if (ho == HIGH) {
        goDown();
        pusherpos = PosControl(limit, pusherpos);
        Serial.println(pusherpos);
        goUp();
        pusherpos = PosControl(nextPos, pusherpos);
        Serial.println(pusherpos);
        checkPos = nextPos - jump;
        Serial.println(checkPos);
        if (checkPos < -1) {
          nextPos = limit - jump;
          moveB(A);
        }
        else {
          nextPos = checkPos;
        }

      }
      else {
        SerialBT.println("haga homing primero");
      }
      break;
    case 'd': //numero de diviciones
      if (val2_int > 3 && val2_int < 10) {
        D = val2_int;
        jump = limit / D;
        SerialBT.println("Cambio realizado");
      }
      else {
        SerialBT.println("valor no valido");
      }
      break;
    case 'a': //avance de banda
      if (val2_int > 10 && val2_int < 100) {
        A = val2_int;
      }
      else {
        SerialBT.println("valor no valido");
      }
      break;
    default:
      SerialBT.println("Instruccion no valida");
  }

}
void move1(int L, int _v0, int _vel, int Pulse, int Dir, int msteps, int Per) {
  double vel = _vel * 0.1047;
  double v0 = _v0 * 0.1047;
  double v = v0;

  //factor es numero de pulsos/rev sobre distancia por revolucion (140)
  double k = msteps * 200 / Per;
  unsigned long N = k * abs(L); //numero de pulsos totales

  if (L >= 0) {
    digitalWrite(Dir, HIGH);
  }
  else if (L < 0) {
    digitalWrite(Dir, LOW);
  }
  //envia pulsos mientras no se alcance en conteo de pulsos
  while (count < N) {
    int N1 = N * 0.3; //pulsos acelerando
    int N2 = N * 0.7; //pulsos desacelerando

    double a = (vel - v0) / N1;
    if (count <= N1) {
      //v = v0 + count*a;
      v = sqrt(v0 * v0 + 2 * a * (count));
    }
    else if (count > N1 & count < N2) {
      v = vel;
    }
    else if (count >= N2) {

      v = sqrt(vel * vel - 2 * a * (count - N2));
      //v[i] = (V0**2 + 2*A*x[i])**(1/2)
      //(V**2 - 2*A*(x[i]-(X-xa)))**(1/2)
    }

    current_time = micros();
    if (v != 0) {

      //digitalWrite(REnPin, LOW); //verificar polaridad
      //digitalWrite(LEnPin, LOW);
      //convertir de rad/s a los tiempo del pulso
      //cada paso (400/giro) tiene 0.9 grados = 0.015708 rad
      //solo es una regla de 3, si debe recorrer X rad en 1 segundo, 0.015708 radianes en cuantos segundos?
      double k2 = 6.28319 / (200 * msteps); //radianes por pulso
      pulse_low_time = (1000000 * k2 / abs(v)) - 100; //debe quedar en micros
      //Serial.println(pulse_low_time);
      //ENVIO DE PULSOS
      if (current_time >= next_time) {
        digitalWrite(Pulse, HIGH);

        delayMicroseconds(100);
        //if(Dir){count +=1;} else {count -= 1;}
        count += 1;
        next_time = micros() + pulse_low_time;
      }
      else if ( current_time < next_time) {
        digitalWrite(Pulse, LOW);

      }
    }
  }

  count = 0;
  digitalWrite(Pulse, LOW);

}
void moveB(int LB) {
  move1(LB, 5, 18, BPul, BDir, Bmsteps, BPer);
}

void moveP(int LP) {
  move1(LP, 100, 200, PPul, PDir, Pmsteps, PPer);
}

void goUp() {
  servo.write(top);
  delay(2000);
}
void goDown() {
  servo.write(down);
  delay(2000);
}

void homing(int Pulse, int Dir, int msteps) {
  goUp();
  int RPM = 30;
  double v = RPM * 0.1047;
  digitalWrite(Dir, LOW);
  double k2 = 6.28319 * 1000000 / (200 * msteps);
  pulse_low_time = (k2 / abs(v)) - 100; //debe quedar en micros
  next_time = micros() + pulse_low_time;
  //envia pulsos hasta que toque el sensor
  Serial.println(pulse_low_time);
  bool sp = digitalRead(EndStop);
  
  while (sp == 1) {
    current_time = micros();
    //ENVIO DE PULSOS
    if (current_time >= next_time) {
      digitalWrite(Pulse, HIGH);
      delayMicroseconds(100);
      next_time = micros() + pulse_low_time;
    }
    else if ( current_time < next_time) {
      digitalWrite(Pulse, LOW);
    }
    sp = digitalRead(EndStop);
  }
  digitalWrite(Pulse, LOW);
  Serial.println(sp);
  delay(1000);

}

int PosControl(int target, int current) {
  //recibe la posicion objetivo y la actual mueve la diferencia
  int delta = target - current;
  //Serial.println(delta);
  if (target >= 0) {
    moveP(delta);
    return target;
  }
  else {
    return current;
  }
}
