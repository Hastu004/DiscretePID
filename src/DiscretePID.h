#ifndef DISCRETE_PID_H
#define DISCRETE_PID_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#endif

namespace discretepid {

enum class Topology : uint8_t {
  Parallel = 0,
  PIDInteracting = 1,
  PI_D = 2,
  I_PD = 3,
};

struct Config {
  float kp = 1.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float sampleTimeSec = 0.02f;
  float outputMin = 0.0f;
  float outputMax = 255.0f;
  float integralMin = -255.0f;
  float integralMax = 255.0f;
  float bias = 0.0f;
  float derivativeFilterAlpha = 0.2f;
  Topology topology = Topology::Parallel;
};

class PIDController {
 public:
  explicit PIDController(const Config& config = Config{});

  void setConfig(const Config& config);
  Config getConfig() const;

  void setTunings(float kp, float ki, float kd);
  void setTopology(Topology topology);
  void setSampleTime(float sampleTimeSec);
  void setOutputLimits(float minOutput, float maxOutput);
  void setIntegralLimits(float minIntegral, float maxIntegral);
  void setBias(float bias);
  void setDerivativeFilterAlpha(float alpha);

  float compute(float setpoint, float input);
  void reset(float output = 0.0f, float input = 0.0f);

  float getLastOutput() const;
  float getIntegralTerm() const;
  float getLastError() const;

 private:
  struct Gains {
    float proportional;
    float integral;
    float derivative;
  };

  Config config_;
  float integralTerm_;
  float filteredInputRate_;
  float lastInput_;
  float lastError_;
  float lastOutput_;
  bool initialized_;

  static float clamp(float value, float minValue, float maxValue);
  Gains currentGains() const;
};

}  // namespace discretepid

#endif
