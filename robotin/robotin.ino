//#include <PID_v2.h>
#include <PID_v1.h>// Libreria que contiene el algoritmo para el computo PID
#include <LMotorController.h> //Libreria para facilitar programacion con el modulo L298N
#include "I2Cdev.h" //Libreria para la comunicación I2C entre arduino y el MPU6050
#include "MPU6050_6Axis_MotionApps20.h" //Libreria para obtener datos del MPU6050
#include <SoftwareSerial.h> //Libreria para la comunicación en serie por otros pines digitales del Arduino

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 30 // velocidad minima de los motores [PWM]

//Para la comunicación bluethoot
#define bluethoot; //si no queremos utilizar bluethoot comentamos esta linea
const int btLED = 4; //led testigo para comprobar funcionamiento de la transimición de datos via bluethoot
const int pinRX = 11;
const int pinTX = 12;
char estado; //almacena un caracter comando para el control bluethoot
SoftwareSerial BT(pinRX,pinTX); //recordar que los pines TX y RX de arduino van conectados al RX y TX del HC05 RESPECTIVAMENTE 

MPU6050 mpu; //creacion del objeto para usar funciones de la libreria del MPU

//################################################################ Parametros PID ################################################################
//---------------Constantes proporcional, derivativa e integral---------------
double Kp = 60;  //double Kp = 60;
double Kd = 3.2;  //double Kd = 2.2; 3.5  
double Ki = 250;  //double Ki = 270;  300            
//---------------Computo PID------------------------------------------  
double originalSetpoint = 0; //si el centro de gravedad de nuestro robot no es siemtrico, ajustamos el punto de equilibrio viendo los valores de pitch del MPU
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output; //definicion de variables donde se almacenaran el input y el output del computo PID 
//---------------Creación de instancia de la clase PID------------------------ 
PID tilt_pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); //(anguloInclinacionActual,,anguloDeInclinaciónDeseado,Kp,Ki,Kd, Direct )
//################################################################ Parametros L298N ################################################################
//---------------Pines para el control de Motores-----------------------------
int ENA = 5;
int IN1 = 7;
int IN2 = 6;
int IN3 = 9;
int IN4 = 8;
int ENB = 10;
//--------------Factores de velocidad-----------------------------------------
double motorSpeedFactorLeft = 0.4;//0.4//los utilizamos para apaciguar la diferencia de RPM que podrian tener ambos motores
double motorSpeedFactorRight = 0.31;//0.31 //los utilizamos para apaciguar la diferencia de RPM que podrian tener ambos motores
//--------------Creación instancia de la clase LMotorController --------------
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight); 

//################################################################ Cuestiones del MPU ################################################################
// Declarando variables de control/estado del MPU
bool dmpReady = false; // set true if DMP init was successful  
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Declarando variables de orientación/movimiento del MPU
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container

volatile bool mpuInterrupt = false; //la definimos como voltatile para indicar al compilador que podria cambiar en cualquier momento y que cuando lo haga debe volver a cargarla
//Funcion que se va a ejecutar durante los interrupts (ISR)                        
void dmpDataReady(){
 mpuInterrupt = true;
}
//################################################################ Funciones Setup y Loop ################################################################
void setup()
{
  Serial.begin(9600);   
  
  #ifdef bluethoot
  BT.begin(38400); //baud rate para la comunicación bluethoot
  #endif 
  init_imu();

}

void loop()
{
  get_MPU_values(); //dentro de la funcion se van actualizando los valores del ángulo de inclinacion que se pasan como input para el computo PID
  
  #ifdef bluethoot
  bluethoot_control();
  #endif
  
  print_val();


}

//################################################################ Funciones para programación en bloques ################################################################

void init_imu() //inizializa el MPU
{
   // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz) Two Wire Baud Register
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif

 //Inicialiamos el MPU y su procesador de movimiento digital DMP
 mpu.initialize();
 devStatus = mpu.dmpInitialize();

 // Offsets obtenidos para el MPU6050, escalados para minima sensibilidad 
 mpu.setXGyroOffset(143); //220
 mpu.setYGyroOffset(-45); //76
 mpu.setZGyroOffset(47);//-85
 mpu.setZAccelOffset(1872); //1788 

 // verificamos que el DMP se haya iniciado (mpu.dmpInitialize() retorna 0 si lo hizo)
 if (devStatus == 0)
 {
  // Activamos el DMP, ahora esta listo
  mpu.setDMPEnabled(true);
  //Activamos la detección de interrupciones de arduino
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  // seteamos la bandera DMP Ready para que la función main loop() sepa que puede usar el DMP
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  
  //setup tilt_PID
  tilt_pid.SetMode(AUTOMATIC);
  tilt_pid.SetSampleTime(10); //El computo PID se actualizará cada 10ms 
  tilt_pid.SetOutputLimits(-255, 255); //establezco el valor maximo y minimo que puede tomar la salida del computo PID
 }
 //si el DMP no se ha iniciado avisamos y mostramos el valor de devStatus
 else
 {
  // ERROR!
  // 1 = initial memory load failed
  // 2 = DMP configuration updates failed
  // (if it's going to break, usually the code will be 1)
  Serial.print(F("DMP Initialization failed (code "));
  Serial.print(devStatus);
  Serial.println(F(")"));
 }
}

void get_MPU_values() //Obtiene los valores sensados por el MPU y procesador por el DMP y se imprimen en el serial monitor o serial plotter
{
  // en caso de que el DMP no este listo para usar, no hacer nada
 if (!dmpReady) return;

 // wait for MPU interrupt or extra packet(s) available
 while (!mpuInterrupt && fifoCount < packetSize)
 {
 //no mpu data - performing PID calculations and output to motors 
 tilt_pid.Compute(); //Al hacer esto obtengo el output del computo PID que fue transformado a un rango entre -255 y 255
 motorController.move(output, MIN_ABS_SPEED); // la velocidad (output) pasa del rango [-255;255] al rango [-255;-MIN_ABS_SPEED]U[MIN_ABS_SPEED;255 ] (output acotado)
 }                                            //y se envian los pulsos PWM esvribiendo en los pines ena y enb y escalando el output acotado para proponer el duty cycle

 // reset interrupt flag and get INT_STATUS byte
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 // get current FIFO count
 fifoCount = mpu.getFIFOCount();

 // check for overflow (this should never happen unless our code is too inefficient)
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 // reset so we can continue cleanly
 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));

 // otherwise, check for DMP data ready interrupt (this should happen frequently)
 }
 else if (mpuIntStatus & 0x02)
 {
 // wait for correct available data length, should be a VERY short wait
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 // read a packet from FIFO
 mpu.getFIFOBytes(fifoBuffer, packetSize);
 
 // track FIFO count here in case there is > 1 packet available
 // (this lets us immediately read more without waiting for an interrupt)
 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI; //el input es el pitch convertido de radianes a grados sexagecimales
 //Serial.println(input);
 }
}

void bluethoot_control() //Configuración para el control bluethoot
{  
  if(BT.available()){        // si hay datos disponibles, los leemos
    estado = BT.read();
  }
  if(estado == 'F'){           // Boton Foward
    setpoint = (originalSetpoint + 1.5);
  }
  else if(estado == 'B'){          // Boton Backward
    setpoint = (originalSetpoint - 1.5); 
  }
  else if(estado == 'L'){          // Boton left
    estado == 'S';
  }
  else if(estado == 'R'){          // Boton right
    estado == 'S';
  } 
  else if(estado == 'S'){         // Boton Parar
    setpoint = originalSetpoint;
  }
  if(estado == 'F'||estado == 'S'||estado == 'B'||estado == 'L' || estado == 'R'){
    digitalWrite(btLED,HIGH);
  }
   if(estado == 'O'){
    digitalWrite(btLED,LOW);
  }
  Serial.println(estado);
}
void print_val(){
  //Serial.println(input); //imprime el pitch (angulo de inclinación)(tambien podria ir dentro de get_MPU_values())
  ///Serial.println(output); //imprime el valor del computo PID - aveces no compila y hay que comentarlo 
  Serial.println(motorController.getCurrentSpeed()); //imprime la velocidad de los motores en PWM
  //Serial.print(originalSetpoint); Serial.print("\t");
  //Serial.print(setpoint); Serial.print("\t");

}
