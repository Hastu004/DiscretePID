<div align="center">

  ### Comparative study of classical PID control algorithms for the angular control of an electromechanical arm

</div>

<div align="center">

![](https://img.shields.io/badge/Contributions-Welcome-brightgreen.svg)
![](https://img.shields.io/badge/Maintained%3F-Yes-brightgreen.svg)

</div>

# DiscretePID

DiscretePID is an Arduino library for classical PID control topologies focused on embedded control loops and actuator experiments.

<p align="center">
  <a href="https://drive.google.com/file/d/16bK7dAosLCTXEhoV1iuj8vgqMc72WxBu/view?usp=sharing">
    <img src="pid-2.gif" alt="Project demo video preview" />
  </a>
</p>

<details>
  <summary>Contents</summary>

  - [Abstract](#abstract)
  - [What changed](#what-changed)
  - [Library structure](#library-structure)
  - [Installation](#installation)
  - [Quick start](#quick-start)
  - [Topologies](#topologies)
  - [Examples](#examples)
  - [Using the examples](#using-the-examples)
  - [Notes about the original sketches](#notes-about-the-original-sketches)
  - [Validation](#validation)
  - [Resources](#resources)
  - [Paper reference](#paper-reference)
  - [License](#license)
</details>

## Abstract

According to various reports, it is estimated that around 70% of industrial control loops use the classical PID control algorithms (Proportional Integral Derivative), the most used of them is the Parallel PID, but there are variants such as serial PID, I-PD and PI-D, each of them has different properties. In this study, the performance of these diverse implementations is evaluated in an angular control plant that consists of an arm manipulated by a Brushless motor. For the angular control of the arm, the various implementations of PID controllers reported in the literature were tested, obtaining the value of their parameters according to the model of the plant. In order to design the algorithms it was necessary to discretize the differential equations that define the PID controllers, transforming their differential equations to equations of differences which were later programmed in C++. In a first instance, the control schemes were implemented in simulation mode to subsequently implement them in real time. The results obtained were quite similar and the subsequent evaluation of the various implementations tested showed that the parallel PID algorithm fulfilled the control objectives more effectively.

Keywords: Automatic control, control algorithms, angular control, PID control.

## What changed

This repository used to contain standalone `.pde` sketches with duplicated logic and several implementation bugs. It is now organized as a reusable Arduino library with:

- A single, consistent API in `src/`
- Support for `Parallel`, `PIDInteracting`, `PI_D`, and `I_PD`
- Output limits and integral clamping
- Basic anti-windup protection
- Derivative action on the measured input to reduce derivative kick
- Ready-to-run examples for simulation and servo-based position control

## Library structure

```text
DiscretePID/
├── examples/
├── src/
├── test/
├── keywords.txt
└── library.properties
```

## Installation

1. Download or clone this repository.
2. Rename the folder to `DiscretePID` if needed.
3. Copy it into your Arduino libraries directory.
4. Restart Arduino IDE.

Typical locations:

- macOS: `~/Documents/Arduino/libraries/`
- Linux: `~/Arduino/libraries/`
- Windows: `Documents/Arduino/libraries/`

## Quick start

```cpp
#include <DiscretePID.h>

discretepid::Config config;
discretepid::PIDController controller;

void setup() {
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
  controller.reset();
}

void loop() {
  float setpoint = 50.0f;
  float measurement = analogRead(A0);
  float output = controller.compute(setpoint, measurement);
  analogWrite(9, (int)output);
  delay(20);
}
```

## Topologies

### `Parallel`

Standard PID using the gains exactly as configured:

`u = bias + Kp * e + integral( Ki * e ) - Kd * d(input)/dt`

### `PIDInteracting`

Series/interacting form using the configured values as the traditional interacting parameters. Internally it is converted to an equivalent implementation:

- Proportional gain: `Kp`
- Integral gain: `Kp * Ki`
- Derivative gain: `Kp * Kd`

### `PI_D`

Proportional and integral action use the error signal. Derivative action is applied to the measured process variable.

### `I_PD`

Integral action uses the error signal, while proportional and derivative action use the measured process variable. This reduces setpoint kick but usually requires more careful tuning.

## Examples

- `examples/BasicPID/BasicPID.ino`: closed-loop numeric demo
- `examples/ServoPositionControl/ServoPositionControl.ino`: adaptation of the original arm/servo experiment
- `examples/TopologyComparison/TopologyComparison.ino`: compares the four topologies on identical simulated plants and prints CSV output for Arduino Serial Plotter

## Using the examples

### `BasicPID`

Use this example to understand the minimum integration flow of the library without any external hardware.

What it does:

- Creates one `PIDController`
- Simulates a simple closed-loop plant
- Prints `setpoint`, `measurement`, and `control` through Serial

How to use it:

1. Open `examples/BasicPID/BasicPID.ino` in Arduino IDE.
2. Compile and upload it to your board.
3. Open Serial Monitor at `115200` baud.
4. Observe how `measurement` approaches `setpoint`.

When to use it:

- To verify the library is installed correctly
- To start tuning without dealing with sensors or actuators
- To understand the basic API: `Config`, `setConfig()`, `reset()`, and `compute()`

### `ServoPositionControl`

Use this example as the starting point for the original electromechanical arm or a similar servo/ESC-based setup.

What it does:

- Reads the angular position from `A0`
- Computes the control output every `20 ms`
- Sends the resulting command in microseconds to the actuator on pin `9`
- Lets you change the setpoint or manually command the actuator over Serial

How to use it:

1. Open `examples/ServoPositionControl/ServoPositionControl.ino`.
2. Confirm your wiring matches the sketch:
   `A0` for the position sensor and pin `9` for the actuator signal.
3. Review and adjust these constants if needed:
   `kSensorRawMin`, `kSensorRawMax`, `kAngleMin`, `kAngleMax`, `kMinSignal`, and `kMaxSignal`.
4. Upload the sketch.
5. Open Serial Monitor at `115200` baud.
6. Send a value from `0` to `180` to change the angular setpoint.
7. Send a value from `1000` to `2000` to directly command the actuator and reset the controller state.

What to tune first:

- `kp`, `ki`, and `kd`
- `bias` for the neutral actuator command
- Output and integral limits
- Sensor mapping range

### `TopologyComparison`

Use this example to compare the behavior of `Parallel`, `PIDInteracting`, `PI_D`, and `I_PD` under the same simulated conditions.

What it does:

- Creates four controllers, one per topology
- Connects each controller to an identical simulated first-order plant
- Applies a step from `0` to `20` after `1 second`
- Prints one CSV line per sample:
  `setpoint,parallel,interacting,pi_d,i_pd`

How to use it:

1. Open `examples/TopologyComparison/TopologyComparison.ino`.
2. Upload the sketch.
3. Open Arduino Serial Plotter at `115200` baud.
4. Watch the four responses evolve after the step change.
5. Open Serial Monitor and send `r` if you want to restart the simulation.

How to read the result:

- A faster rise means the controller reacts more aggressively.
- More overshoot means the response exceeds the setpoint before settling.
- A smoother curve usually means less aggressive derivative/proportional action.
- Differences between curves show how each topology distributes proportional, integral, and derivative action.

Important note:

- This example is only for visualization and learning.
- It does not represent your real plant until you tune the gains and replace the simulated plant with your hardware.

## Notes about the original sketches

The original `.pde` files are still present in the repository as historical reference from the thesis project, but the maintained implementation is the library in `src/`.

## Validation

The repository includes a native smoke test in `test/native_smoke_test.cpp` so the control code can be compiled and sanity-checked outside Arduino IDE.

## Resources

- Paper (PDF): https://scielo.conicyt.cl/pdf/ingeniare/v28n4/0718-3305-ingeniare-28-04-612.pdf
- Paper (article): https://www.scielo.cl/scielo.php?pid=S0718-33052020000400612&script=sci_arttext
- Demo video: https://drive.google.com/file/d/16bK7dAosLCTXEhoV1iuj8vgqMc72WxBu/view?usp=sharing

## Paper reference

Hernan Astudillo, "Estudio comparativo de algoritmos de control PID clasicos para el control angular de un brazo electromecanico", Ingeniare. Revista chilena de ingenieria, vol. 28, no. 4, 2020, pp. 612-623.

## License

MIT. See `LICENSE`.
