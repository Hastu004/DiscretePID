#include <DiscretePID.h>
#include <Servo.h>

namespace {

constexpr uint8_t kMotorPin = 9;
constexpr uint8_t kSensorPin = A0;
constexpr int kMinSignal = 1000;
constexpr int kMaxSignal = 2000;
constexpr float kSampleTimeSec = 0.02f;
constexpr unsigned long kSampleTimeMs = 20;
constexpr long kSensorRawMin = 419;
constexpr long kSensorRawMax = 943;
constexpr long kAngleMin = -25;
constexpr long kAngleMax = 113;
constexpr discretepid::Topology kTopology = discretepid::Topology::Parallel;

Servo motor;
discretepid::PIDController controller;
float setpointDeg = 45.0f;
unsigned long lastUpdateMs = 0;

float readAngleDegrees() {
  const long raw = analogRead(kSensorPin);
  return static_cast<float>(map(raw, kSensorRawMin, kSensorRawMax, kAngleMax, kAngleMin));
}

void applyManualCommand(int command) {
  if (command >= kMinSignal && command <= kMaxSignal) {
    motor.writeMicroseconds(command);
    controller.reset(static_cast<float>(command), readAngleDegrees());
  } else if (command >= 0 && command <= 180) {
    setpointDeg = static_cast<float>(command);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);

  motor.attach(kMotorPin, kMinSignal, kMaxSignal);
  motor.writeMicroseconds(kMinSignal);
  delay(1500);

  discretepid::Config config;
  config.kp = 5.0f;
  config.ki = 8.5f;
  config.kd = 0.03f;
  config.sampleTimeSec = kSampleTimeSec;
  config.outputMin = kMinSignal;
  config.outputMax = kMaxSignal;
  config.integralMin = -350.0f;
  config.integralMax = 350.0f;
  config.bias = 1500.0f;
  config.derivativeFilterAlpha = 0.35f;
  config.topology = kTopology;

  controller.setConfig(config);
  controller.reset(1500.0f, readAngleDegrees());

  Serial.println(F("DiscretePID"));
  Serial.println(F("Send 0-180 to change setpoint in degrees."));
  Serial.println(F("Send 1000-2000 to command the ESC manually."));
}

void loop() {
  if (Serial.available() > 0) {
    applyManualCommand(Serial.parseInt());
  }

  const unsigned long now = millis();
  if (now - lastUpdateMs < kSampleTimeMs) {
    return;
  }
  lastUpdateMs = now;

  const float angle = readAngleDegrees();
  const float controlSignal = controller.compute(setpointDeg, angle);

  motor.writeMicroseconds(static_cast<int>(controlSignal));

  Serial.print("setpoint=");
  Serial.print(setpointDeg);
  Serial.print(", angle=");
  Serial.print(angle);
  Serial.print(", output=");
  Serial.println(controlSignal);
}
