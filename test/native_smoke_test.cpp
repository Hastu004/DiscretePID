#include "DiscretePID.h"

#include <cstdlib>
#include <iostream>

int main() {
  discretepid::Config config;
  config.kp = 2.0f;
  config.ki = 0.5f;
  config.kd = 0.1f;
  config.sampleTimeSec = 0.02f;
  config.outputMin = -100.0f;
  config.outputMax = 100.0f;
  config.integralMin = -20.0f;
  config.integralMax = 20.0f;

  discretepid::PIDController controller(config);
  controller.reset(0.0f, 0.0f);

  float measurement = 0.0f;
  for (int index = 0; index < 200; ++index) {
    const float output = controller.compute(10.0f, measurement);
    measurement += output * 0.01f;
  }

  const float finalError = controller.getLastError();
  std::cout << "final_error=" << finalError << "\n";

  return ((finalError < -10.0f) || (finalError > 10.0f)) ? EXIT_FAILURE : EXIT_SUCCESS;
}
