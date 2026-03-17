#include <DiscretePID.h>

namespace {

constexpr float kSampleTimeSec = 0.02f;
constexpr unsigned long kSampleTimeMs = 20;
constexpr float kStepSetpoint = 20.0f;
constexpr float kInitialSetpoint = 0.0f;

struct PlantState {
  float output = 0.0f;
};

struct ComparisonLoop {
  const char* name;
  discretepid::PIDController controller;
  PlantState plant;
};

float updatePlant(PlantState& plant, float controlSignal) {
  // Simple first-order plant used only for visualization.
  plant.output += (controlSignal - plant.output) * 0.04f;
  return plant.output;
}

discretepid::Config makeConfig(discretepid::Topology topology) {
  discretepid::Config config;
  config.kp = 2.0f;
  config.ki = 0.7f;
  config.kd = 0.12f;
  config.sampleTimeSec = kSampleTimeSec;
  config.outputMin = -100.0f;
  config.outputMax = 100.0f;
  config.integralMin = -30.0f;
  config.integralMax = 30.0f;
  config.topology = topology;
  return config;
}

ComparisonLoop loops[] = {
    {"parallel", discretepid::PIDController(makeConfig(discretepid::Topology::Parallel)), {}},
    {"interacting", discretepid::PIDController(makeConfig(discretepid::Topology::PIDInteracting)), {}},
    {"pi_d", discretepid::PIDController(makeConfig(discretepid::Topology::PI_D)), {}},
    {"i_pd", discretepid::PIDController(makeConfig(discretepid::Topology::I_PD)), {}},
};

float setpoint = kInitialSetpoint;
unsigned long lastUpdateMs = 0;
unsigned long startMs = 0;
bool stepApplied = false;

void resetSimulation() {
  setpoint = kInitialSetpoint;
  stepApplied = false;
  startMs = millis();

  for (ComparisonLoop& loop : loops) {
    loop.plant.output = 0.0f;
    loop.controller.reset(0.0f, 0.0f);
  }
}

void printHelp() {
  Serial.println(F("DiscretePID - TopologyComparison"));
  Serial.println(F("This example compares the 4 PID topologies using the same simulated plant."));
  Serial.println(F("Open Serial Plotter at 115200 baud to visualize the response."));
  Serial.println(F("A step from 0 to 20 is applied after 1 second."));
  Serial.println(F("Open Serial Monitor and send 'r' to restart the simulation."));
  Serial.println(F("Columns: setpoint, parallel, interacting, pi_d, i_pd"));
}

void printHeader() {
  Serial.println(F("setpoint,parallel,interacting,pi_d,i_pd"));
}

void handleSerial() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'r' || command == 'R') {
      resetSimulation();
      printHeader();
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  printHelp();
  resetSimulation();
  printHeader();
}

void loop() {
  handleSerial();

  const unsigned long now = millis();
  if (now - lastUpdateMs < kSampleTimeMs) {
    return;
  }
  lastUpdateMs = now;

  if (!stepApplied && (now - startMs >= 1000UL)) {
    setpoint = kStepSetpoint;
    stepApplied = true;
  }

  Serial.print(setpoint);

  for (ComparisonLoop& loop : loops) {
    const float controlSignal = loop.controller.compute(setpoint, loop.plant.output);
    updatePlant(loop.plant, controlSignal);

    Serial.print(',');
    Serial.print(loop.plant.output);
  }

  Serial.println();
}
