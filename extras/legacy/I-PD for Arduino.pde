#include <Servo.h>
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 9

int lectura=0;
double lecturaPotenciometro=0;
Servo motor; 
int T=20; 

double Kp=5;
double Ki=8.5;
double Kd=0.03;
double setPoint=45;

float ei;
float e[3];
float u;
float uAnterior=1000;
float Tm=0.02;

float b0= Ki*Tm;
float b1= -Kp - Kd/Tm + Ki*Tm;
float b2= 2*Kd/Tm + Kp;
float b3= -Kd/Tm;

int Yik=0;
int Yik1=0;
int Yk=0;
int Yk1=0;
int Yk2=0;

void setup() {
  Serial.begin(9600);
  delay(1500);
  motor.attach(MOTOR_PIN);
  motor.writeMicroseconds(MAX_SIGNAL);
  motor.writeMicroseconds(MIN_SIGNAL);
  while (!Serial.available()){}
}
void loop() {
  if (Serial.available() > 0) {
    lectura = Serial.parseInt();
    if (lectura > 999 and lectura < 1500) {
      motor.writeMicroseconds(lectura);
    }     
        if (lectura > 0 and lectura < 90)
    {
      setPoint=lectura;
    }    
  }
    lecturaPotenciometro = analogRead(0);
    lecturaPotenciometro = map(lecturaPotenciometro,419,943,113,-25);
    Serial.println(lecturaPotenciometro);
    ei=setPoint-lecturaPotenciometro;
    u[0]= b0*Yik + b1*Yk + b2*Yk1 + b3*Yk2 + uAnterior;
  
    e[1]=e[0];
    e[0]=ei;
    uAnterior=u;
    motor.writeMicroseconds(u);
    delay(T);
}
