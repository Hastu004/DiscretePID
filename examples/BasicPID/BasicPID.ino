#include <DiscretePID.h>

discretepid::Config config;
discretepid::PIDController controller;

float setpoint = 50.0f;
float measurement = 0.0f;

void setup() {
  Serial.begin(115200);

  config.kp = 2.2f;
  config.ki = 0.8f;
  config.kd = 0.12f;
  config.sampleTimeSec = 0.02f;
  config.outputMin = 0.0f;
  config.outputMax = 255.0f;
  config.integralMin = -80.0f;
  config.integralMax = 80.0f;
  config.topology = discretepid::Topology::Parallel;

  controller.setConfig(config);
  controller.reset(0.0f, measurement);
}

void loop() {
  const float control = controller.compute(setpoint, measurement);

  Serial.print("setpoint=");
  Serial.print(setpoint);
  Serial.print(", measurement=");
  Serial.print(measurement);
  Serial.print(", control=");
  Serial.println(control);

  measurement += (control - measurement) * 0.05f;
  delay(20);
}
