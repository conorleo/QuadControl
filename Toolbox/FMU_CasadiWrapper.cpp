#include "FMU_CasadiWrapper.hpp"
#include <algorithm>
#include <cstring>

FMU_CasadiWrapper::FMU_CasadiWrapper(int argc, char** argv, fmi2Type type,
                                     const fmi2ValueReference* thrustVr, std::size_t nThrust,
                                     const fmi2Real* thrusts,
                                     const fmi2ValueReference* bodyVr, std::size_t nBody) {
  m_thrustVr = thrustVr;
  m_nThrust = nThrust;
  m_bodyVr = bodyVr;
  m_nBody = nBody;

  const fmi2Boolean loggingOn = (argc > 2 && std::strcmp(argv[2], "--verbose") == 0) ? fmi2True
                               : (argc > 1 && std::strcmp(argv[1], "--verbose") == 0) ? fmi2True
                                                                                     : fmi2False;
  const fmi2Real tStart = 0.0;
  const fmi2Real* tStop = nullptr;
  m_fm = Initialise(argc, argv, type, tStart, tStop, loggingOn, m_thrustVr, m_nThrust, thrusts);

  // For now we don't try to discover nx/nu from the model. Caller must know
  // sizes. Set defaults to common values; users should set/check these.
  m_nx = 12;
  m_nu = (m_nThrust > 0) ? m_nThrust : 4;
}

FMU_CasadiWrapper::~FMU_CasadiWrapper() {
  Terminate(m_fm);
}

bool FMU_CasadiWrapper::eval(const std::vector<double>& x, const std::vector<double>& u, double t,
                              std::vector<double>& dx) {
  if (x.size() != m_nx) return false;
  if (u.size() != m_nu) return false;

  dx.assign(m_nx, 0.0);

  // Prepare arrays
  std::vector<fmi2Real> states(m_nx);
  std::vector<fmi2Real> controls(m_nu);
  for (std::size_t i = 0; i < m_nx; ++i) states[i] = x[i];
  for (std::size_t i = 0; i < m_nu; ++i) controls[i] = u[i];
  std::vector<fmi2Real> derivatives(m_nx, 0.0);

  // Set controls if available
  if (m_thrustVr && m_nThrust > 0) {
    // Ensure we have m_nThrust == m_nu or copy as many as possible
    std::size_t nset = std::min(m_nThrust, m_nu);
    check(m_fm.fmi2SetReal(m_fm.c, m_thrustVr, nset, controls.data()), "fmi2SetReal(thrust)");
  }

  // Set time and states then request derivatives
  check(m_fm.fmi2SetTime(m_fm.c, t), "fmi2SetTime");
  check(m_fm.fmi2SetContinuousStates(m_fm.c, states.data(), m_nx), "fmi2SetContinuousStates");
  check(m_fm.fmi2GetDerivatives(m_fm.c, derivatives.data(), m_nx), "fmi2GetDerivatives");

  for (std::size_t i = 0; i < m_nx; ++i) dx[i] = derivatives[i];
  return true;
}
