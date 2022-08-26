#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

/* -----------------------------------------------------------
   CONFIGURACION Bluetooth
  ------------------------------------------------------------*/
// Check if Bluetooth configs are enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// Handle received and sent messages
String message = "";
char incomingChar;
boolean inputComplete = false;

//variables programables por BTserial
int F = 5; //frecuencia en minutos

//variables leidas por BTserial
int A_AC; //amperaje en AC
int A_DC;  //amperaje en DC
int A_M;  //amperaje del motor

/* -----------------------------------------------------------
   Pines y parametros para el driver TMC2130
  ------------------------------------------------------------*/

#define EN_PIN           21 // Enable
#define DIR_PIN          0 // Direction
#define STEP_PIN         4 // Step
#define PS_PIN           22 // power suppy for driver
#define CS_PIN           5 // Chip select
#define SW_MOSI          23 // Software Master Out Slave In (MOSI)
#define SW_MISO          19 // Software Master In Slave Out (MISO)
#define SW_SCK           18 // Software Slave Clock (SCK)
#define R_SENSE 0.11f
TMC2130Stepper driver = TMC2130Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
using namespace TMC2130_n;
bool dir = true;
bool shaft = false, flag1 = false;
bool error_flag = false;

int STALL_VALUE  =     0; // [-64..63]
int RMSC        =      300; //CORRIENTE

/* -----------------------------------------------------------
   Variables globales para tiempo
  ------------------------------------------------------------*/
unsigned long ct,nt;  //ct: current time, nt: next time
unsigned long minute = 60000; // 60000 milliseconds in a minute
unsigned long second = 1000; // 60000 milliseconds in a minute
int period_min = 1; //PERIODO DE ACTIVACION DEL BARREDOR EN MINUTOS
unsigned long period = period_min*minute; ////PERIODO DE ACTIVACION DEL BARREDOR EN MILISEGUNDOS

void setup() {
  
  pinMode(CS_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PS_PIN, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  Serial.begin(250000);
  SerialBT.begin("BIOTA1");
  SerialBT.println("ok");
  Serial.println("ok");
  reset();
  //sweep(false,true);
  nt = 0;
}

void loop() {
  
  //mover barredor cada N minutos
  ct = millis();
  if ((ct-nt) > period){
      nt = ct;
      sweep(true,true);
      delay(1000);
      sweep(false,true);
  }
  
  //configurar por bluetooh
  serialEvent();
  if (inputComplete) {
  console(message);
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

void console(String val) { // d8 home h feed
  char valchar = val.charAt(0); // varchar = "f"
  String val2 = val.substring(1); //val2 = "fff"
  int val2_int = val2.toInt();  // val2_int = 4325435
  switch (valchar) {
    case 'm':
      sweep(true,true);
      delay(1000);
      sweep(false,true);
      break;
    case 'p':
      if (val2_int > 5 && val2_int < 60) {
         F = val2_int;
      }
      break;
    case 'r':
      SerialBT.println("reset");
      reset();
      break;
    case 'c':
      SerialBT.println("check");
      check();
      break;
    case 's':
      STALL_VALUE = val2_int;
      break;
    case 'a':
      RMSC = val2_int;
      break;
    case 't':
      period_min = val2_int;
      period = period_min*minute;
    default:
      SerialBT.println("Instruccion no valida");
  }
}

void sweep(bool dir, bool vel){
  digitalWrite(EN_PIN, LOW);
  delay(200);
  unsigned long last_time = micros();
  unsigned long checkpoint_time = micros();
  unsigned long ms = micros();
  DRV_STATUS_t drv_status{0};
  drv_status.sr = driver.DRV_STATUS();
  int sg = 200;
  uint32_t low_time;
  if (vel){
    low_time = 600;
  }
  else{
    low_time = 800;
  }
  int sum = 0;
  int sg2 = 0;
  driver.shaft(dir);
  driver.rms_current(RMSC); // mA
  driver.sgt(STALL_VALUE);
  while (!(sg2>2 && sum>20)){
      ms = micros();
      if((ms-last_time) > low_time) { 
        last_time = ms;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(STEP_PIN, LOW);
        drv_status.sr = driver.DRV_STATUS();
        sg = drv_status.sg_result;
        if (sg < 70){
          sg2 = sg2+1;
        }
        else{
          sg2 = 0;
        }
        //Serial.println(sg);
        sum = sum + 1;
      } 
      if((ms-checkpoint_time) > 7*1000000 || sum == 9950) {
        //SerialBT.println("error");
        Serial.println("error");
        break;
      }
      
  }
  Serial.println(sum);
  digitalWrite(EN_PIN, HIGH);

}

bool check(){
    DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();
    int coil1 = drv_status.s2gb;
    int coil2 = drv_status.s2ga;
    A_M = driver.cs2rms(drv_status.cs_actual);
    SerialBT.print(coil1, DEC);
    SerialBT.print(" ");
    SerialBT.print(coil2, DEC);
    SerialBT.print(" ");
    SerialBT.println(A_M, DEC);
    Serial.print(coil1, DEC);
    Serial.print(" ");
    Serial.print(coil2, DEC);
    Serial.print(" ");
    Serial.println(A_M, DEC);
    if (coil1 || coil2){
        error_flag = true;
    }
    else {
        error_flag = false;
    }
}

void reset(){
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(CS_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(PS_PIN, LOW);
    digitalWrite(EN_PIN, HIGH);
    delay(1000);
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(CS_PIN, HIGH);
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(PS_PIN, HIGH);
    digitalWrite(EN_PIN, LOW);
    driver.begin();
    driver.rms_current(RMSC); // mA
    driver.sgt(STALL_VALUE);
    driver.microsteps(2);
    driver.intpol(1);
    driver.toff(5);
    driver.chm(0);
    driver.hstrt(0);
    driver.hend(0);
    driver.tbl(2);
    driver.en_pwm_mode(0);
    driver.pwm_autoscale(1);
    driver.pwm_freq(0b00);
    //driver.pwm_ampl(200);
    //driver.pwm_grad(1);      
    driver.irun(25);
    driver.ihold(10);
    driver.iholddelay(5);
    driver.TPOWERDOWN(50);
    driver.blank_time(24);
    driver.TPWMTHRS(60); 
    driver.TCOOLTHRS(50); // 20bit max
    driver.THIGH(40);
    //driver.semin(1);
    //driver.semax(14);
    //driver.sedn(2);
    
    digitalWrite(EN_PIN, HIGH);           
}
