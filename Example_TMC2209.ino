
#include <TMCStepper.h>
#include "TeensyStep.h"
#define MAX_SPEED        5 // In timer value
#define MIN_SPEED      1000
#define STALL_VALUE     0 // [0..255]
#define EN_PIN           4 // Enable
#define DIR_PIN          3 // Direction
#define STEP_PIN         2 // Step
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
int RMSC = 1400; // corriente rms del motor




#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

//dos objetos con los que vamos a trabajar:
//1. el driver es para comunicarnos con el driver TMC y configurar sus funciones o leer las varables
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
//2. el motor y controller son de la liberia teensystep para manejar la cinematica y los pulsos al driver. 
Stepper motor(STEP_PIN, DIR_PIN);       // STEP pin: 2, DIR pin: 3
StepControl controller;    // Use default settings 


using namespace TMC2209_n;
void setup() {
  Serial.begin(250000);         // Init serial port and set baudrate
  //while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  SERIAL_PORT.begin(115200);  //este se usa con hardware serial
  //driver.beginSerial(115200);  //este solo se usa con software serial
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(600); // mA
  driver.microsteps(8);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  // con estas dos lineas se activa el StallGuard, la condicion es TCOOLTHRS ≥ TSTEP > TPWMTHRS, como no sabemos el TSTEP pues ponemos unos limites muy grandes y sale
  driver.TPWMTHRS(60);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.SGTHRS(STALL_VALUE);
  driver.en_spreadCycle(false);   //spreadCycle
  driver.pwm_autoscale(true); //stealthChop
  motor
    .setMaxSpeed(3000)       // steps/s
    .setAcceleration(100000); // steps/s^2 

}

void loop() {
  homing(LOW);
  //despues de hacer el homing puede hacer los movimientos
  delay(2000);

  //este es un ejemplo del movimiento, que debemos hacer? --> crear una funcion de cinematica inversa y otra que la use para hacer los movimientos articulares.
  motor.setTargetRel(20000);  // Set target position to 1000 steps from current position
  controller.move(motor);    // Do the move
  delay(1000);
}

//void moveit()
void homing(bool dir){

  //habilitamos el motor
  digitalWrite(EN_PIN, LOW);
  delay(200);
  unsigned long last_time = micros();
  unsigned long ms = micros();
  int sg = 200;
  uint32_t low_time;
  driver.shaft(dir); //cambiamos la direccion por software, pueden hacerlo tambien desde el pin DIR de cada motor
  low_time = 100;
  int sum =0;
  int pul_count = 0;
  while (!(sum>=1)){  //si se detectan 5 choques, se para y hay homing
      ms = micros();
      if((ms-last_time) > low_time) { 
        last_time = ms;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(STEP_PIN, LOW);
        sg = driver.SG_RESULT();
        Serial.println(sg, DEC);  //activar si quieren ver el valor del StallGuard
        //Serial.print("TSTEP: ");
        if (pul_count < 20){pul_count = pul_count +1;}  //esto es solo para evitar que se detecte el homing por la inercial de arranque, los primeros pulsos no suman
        
        if (sg < 5 && pul_count>10){  //si se detecta que hay choque, se suma uno, para esto se lee sg-result si es menor a 5 es porque hay choque, se puede cambiar el nivel
          sum = sum + 1; //esto es para que no sea tan sensible la deteccion de stall, tiene que contar n pasos en stall para que salga del bucle
        }
        else{
          sum = 0;
        }
      }        
  }
  Serial.println("homing done");
  motor.setPosition(0); //puede ser otra posicion inicial, pero se debe setear para poder usar target adsoluto en adelante.
}
