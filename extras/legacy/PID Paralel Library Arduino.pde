#include <PID_v1.h>
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
double salida;

PID myPID(&lecturaPotenciometro, &salida,&setPoint,Kp,Ki,Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  delay(1500);
  motor.attach(MOTOR_PIN);
  motor.writeMicroseconds(MAX_SIGNAL);
  motor.writeMicroseconds(MIN_SIGNAL);
  delay(1500);
  myPID.SetMode(AUTOMATIC);
  while (!Serial.available()){}
}
void loop() {
  if (Serial.available() > 0) {
    lectura = Serial.parseInt();
    if (lectura > 999 and lectura < 1500) {
      motor.writeMicroseconds(lectura);
    }     
    if (lectura > 0 and lectura < 90) {
      setPoint=lectura;
    }    
  }
  lecturaPotenciometro = analogRead(0);
  lecturaPotenciometro = map(lecturaPotenciometro,419,943,113,-25);
  Serial.println(lecturaPotenciometro);
  myPID.Compute();
  motor.writeMicroseconds(salida);
  delay(T);
}
