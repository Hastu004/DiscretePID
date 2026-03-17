#include "DiscretePID.h"

namespace discretepid {

PIDController::PIDController(const Config& config)
    : config_(config),
      integralTerm_(0.0f),
      filteredInputRate_(0.0f),
      lastInput_(0.0f),
      lastError_(0.0f),
      lastOutput_(config.bias),
      initialized_(false) {
  setConfig(config);
}

void PIDController::setConfig(const Config& config) {
  config_ = config;

  if (config_.sampleTimeSec <= 0.0f) {
    config_.sampleTimeSec = 0.02f;
  }

  if (config_.outputMin > config_.outputMax) {
    const float previousMin = config_.outputMin;
    config_.outputMin = config_.outputMax;
    config_.outputMax = previousMin;
  }

  if (config_.integralMin > config_.integralMax) {
    const float previousMin = config_.integralMin;
    config_.integralMin = config_.integralMax;
    config_.integralMax = previousMin;
  }

  setDerivativeFilterAlpha(config_.derivativeFilterAlpha);
  integralTerm_ = clamp(integralTerm_, config_.integralMin, config_.integralMax);
  lastOutput_ = clamp(lastOutput_, config_.outputMin, config_.outputMax);
}

Config PIDController::getConfig() const {
  return config_;
}

void PIDController::setTunings(float kp, float ki, float kd) {
  config_.kp = kp;
  config_.ki = ki;
  config_.kd = kd;
}

void PIDController::setTopology(Topology topology) {
  config_.topology = topology;
}

void PIDController::setSampleTime(float sampleTimeSec) {
  if (sampleTimeSec > 0.0f) {
    config_.sampleTimeSec = sampleTimeSec;
  }
}

void PIDController::setOutputLimits(float minOutput, float maxOutput) {
  config_.outputMin = minOutput;
  config_.outputMax = maxOutput;
  if (config_.outputMin > config_.outputMax) {
    const float previousMin = config_.outputMin;
    config_.outputMin = config_.outputMax;
    config_.outputMax = previousMin;
  }

  lastOutput_ = clamp(lastOutput_, config_.outputMin, config_.outputMax);
}

void PIDController::setIntegralLimits(float minIntegral, float maxIntegral) {
  config_.integralMin = minIntegral;
  config_.integralMax = maxIntegral;
  if (config_.integralMin > config_.integralMax) {
    const float previousMin = config_.integralMin;
    config_.integralMin = config_.integralMax;
    config_.integralMax = previousMin;
  }

  integralTerm_ = clamp(integralTerm_, config_.integralMin, config_.integralMax);
}

void PIDController::setBias(float bias) {
  config_.bias = bias;
}

void PIDController::setDerivativeFilterAlpha(float alpha) {
  if (alpha < 0.0f) {
    config_.derivativeFilterAlpha = 0.0f;
  } else if (alpha > 0.99f) {
    config_.derivativeFilterAlpha = 0.99f;
  } else {
    config_.derivativeFilterAlpha = alpha;
  }
}

float PIDController::compute(float setpoint, float input) {
  const float dt = config_.sampleTimeSec;
  const float error = setpoint - input;

  if (!initialized_) {
    lastInput_ = input;
    lastError_ = error;
    lastOutput_ = clamp(config_.bias, config_.outputMin, config_.outputMax);
    filteredInputRate_ = 0.0f;
    initialized_ = true;
    return lastOutput_;
  }

  const Gains gains = currentGains();
  const float rawInputRate = (input - lastInput_) / dt;
  filteredInputRate_ =
      (config_.derivativeFilterAlpha * filteredInputRate_) +
      ((1.0f - config_.derivativeFilterAlpha) * rawInputRate);

  float proportionalTerm = 0.0f;
  if (config_.topology == Topology::I_PD) {
    proportionalTerm = -gains.proportional * input;
  } else {
    proportionalTerm = gains.proportional * error;
  }

  const float derivativeTerm = -gains.derivative * filteredInputRate_;
  const float candidateIntegral = clamp(
      integralTerm_ + (gains.integral * error * dt),
      config_.integralMin,
      config_.integralMax);

  float unclampedOutput =
      config_.bias + proportionalTerm + candidateIntegral + derivativeTerm;
  float output = clamp(unclampedOutput, config_.outputMin, config_.outputMax);

  const bool saturatedHigh = output >= config_.outputMax;
  const bool saturatedLow = output <= config_.outputMin;
  const bool drivesFurtherHigh = saturatedHigh && (error > 0.0f);
  const bool drivesFurtherLow = saturatedLow && (error < 0.0f);

  if (drivesFurtherHigh || drivesFurtherLow) {
    unclampedOutput =
        config_.bias + proportionalTerm + integralTerm_ + derivativeTerm;
    output = clamp(unclampedOutput, config_.outputMin, config_.outputMax);
  } else {
    integralTerm_ = candidateIntegral;
  }

  lastInput_ = input;
  lastError_ = error;
  lastOutput_ = output;
  return lastOutput_;
}

void PIDController::reset(float output, float input) {
  integralTerm_ = 0.0f;
  filteredInputRate_ = 0.0f;
  lastInput_ = input;
  lastError_ = 0.0f;
  lastOutput_ = clamp(output, config_.outputMin, config_.outputMax);
  initialized_ = true;
}

float PIDController::getLastOutput() const {
  return lastOutput_;
}

float PIDController::getIntegralTerm() const {
  return integralTerm_;
}

float PIDController::getLastError() const {
  return lastError_;
}

float PIDController::clamp(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

PIDController::Gains PIDController::currentGains() const {
  if (config_.topology == Topology::PIDInteracting) {
    return Gains{config_.kp, config_.kp * config_.ki, config_.kp * config_.kd};
  }

  return Gains{config_.kp, config_.ki, config_.kd};
}

}  // namespace discretepid
