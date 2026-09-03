// FMI 2.0 co-simulation host for the Quadcopter FMU.
// Does not modify model sources. Load the unzipped FMU (or pass its path) and step.

#include "../../Toolbox/FMU.h"

// From modelDescription.xml (Quadcopter FMI 2.0)
static const fmi2ValueReference kThrustVr[4] = {348, 349, 350, 351};
static const fmi2ValueReference kBodyR0Vr[3] = {3, 4, 5};

int main(int argc, char** argv) {
  const fmi2Real tStart = 0.0;
  const fmi2Real tStop = 3.0;
  const fmi2Real dt = 0.1;
  const fmi2Real thrust = -30.0;

  std::cout << "Co-sim " << tStart << " -> " << tStop << " s, dt=" << dt
            << ", thrust1-4=" << thrust << "\n";

  const fmi2Real thrusts[4] = {thrust, thrust, thrust, thrust};

  const fmi2Boolean loggingOn = (argc > 2 && std::strcmp(argv[2], "--verbose") == 0) ? fmi2True
                               : (argc > 1 && std::strcmp(argv[1], "--verbose") == 0) ? fmi2True
                                                                                     : fmi2False;

  FMU fm = Initialise(argc, argv, fmi2CoSimulation, tStart, tStop, loggingOn, kThrustVr, 4,
                       thrusts);

  std::printf("%10s %12s %12s %12s\n", "t", "x", "y", "z");
  fmi2Real pos[3]{};
  check(fm.fmi2GetReal(fm.c, kBodyR0Vr, 3, pos), "fmi2GetReal(body.r_0)");
  std::printf("%10.4f %12.6f %12.6f %12.6f\n", tStart, pos[0], pos[1], pos[2]);

  fmi2Real time = tStart;
  int printEvery = static_cast<int>(std::lround(0.1 / dt));
  int step = 0;
  while (time + dt <= tStop) {
    StepCoSimulation(fm, time, dt, kThrustVr, 4, thrusts, kBodyR0Vr, 3, pos);
    time += dt;
    ++step;
    if (step % printEvery == 0 || time + dt > tStop) {
      std::printf("%10.4f %12.6f %12.6f %12.6f\n", time, pos[0], pos[1], pos[2]);
    }
  }

  Terminate(fm);
  std::cout << "Done.\n";
  return 0;
}
