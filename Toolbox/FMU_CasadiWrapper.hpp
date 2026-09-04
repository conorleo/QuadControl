#pragma once

#include "FMU.h"
#include <vector>
#include <string>

// Simple numeric wrapper around the FMU to evaluate derivatives/outputs
// from C++ code. Not a CasADi Callback subclass, but provides a small
// API that can be used to build one later.

class FMU_CasadiWrapper {
 public:
  // Construct by passing command-line args (same as Initialise). Optionally
  // pass value references for thrust and body outputs used by the model.
  FMU_CasadiWrapper(int argc, char** argv, fmi2Type type = fmi2ModelExchange,
                    const fmi2ValueReference* thrustVr = nullptr, std::size_t nThrust = 0,
                    const fmi2Real* thrusts = nullptr,
                    const fmi2ValueReference* bodyVr = nullptr, std::size_t nBody = 0);
  ~FMU_CasadiWrapper();

  // Evaluate derivatives at given time/state/control. Inputs are plain
  // vectors (size checks are performed). The method will NOT advance the
  // FMU time or change internal states permanently.
  // - x: size nx
  // - u: size nu
  // - t: time
  // - dx (output): size nx
  bool eval(const std::vector<double>& x, const std::vector<double>& u, double t,
            std::vector<double>& dx);

  // Convenience accessors
  std::size_t nx() const { return m_nx; }
  std::size_t nu() const { return m_nu; }

 private:
  FMU m_fm;
  std::size_t m_nx = 0;
  std::size_t m_nu = 0;
  const fmi2ValueReference* m_thrustVr = nullptr;
  std::size_t m_nThrust = 0;
  const fmi2ValueReference* m_bodyVr = nullptr;
  std::size_t m_nBody = 0;
};
