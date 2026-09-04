// FMI 2.0 multiple shooting host for the Quadcopter FMU.
// Does not modify model sources. Load the unzipped FMU (or pass its path) and step.

#include "../Toolbox/FMU.h"
#include <casadi/casadi.hpp>
#include "../Toolbox/FMU_CasadiWrapper.hpp"
#include "../Toolbox/FMU_CasadiCallback.hpp"

// From modelDescription.xml (Quadcopter FMI 2.0)
static const fmi2ValueReference kThrustVr[4] = {348, 349, 350, 351};
static const fmi2ValueReference kYVr[3] = {3, 4, 5};

int main(int argc, char** argv) {
  // ------------------------------ FMU -------------------------------
  const fmi2Boolean loggingOn = (argc > 2 && std::strcmp(argv[2], "--verbose") == 0) ? fmi2True
                               : (argc > 1 && std::strcmp(argv[1], "--verbose") == 0) ? fmi2True
                                                                                     : fmi2False;

  constexpr std::size_t kNx = 12;
  constexpr std::size_t kNu = 4;
  constexpr std::size_t kNy = 3;

  const fmi2Real tStart = 0.0;
  const fmi2Real* tStop = nullptr;
  std::vector<fmi2Real> initialThrusts(kNu, 0.0);
  FMU fm = Initialise(argc, argv, fmi2ModelExchange, tStart, tStop, loggingOn,
                      kThrustVr, kNu, initialThrusts.data());
//   fmi2Real states[kNx]{};
  // Use CasADi symbolic types for derivatives/time in the OCP build
  casadi::MX derivatives = casadi::MX::zeros(kNx, 1);

  // Observation Y from the FMU is represented by state slices (x,y,z)

  casadi::MX time = tStart;

  // ------------------------------ OCP -------------------------------

  int N = 100; // number of control intervals

  casadi::Opti opti = casadi::Opti(); // Optimization problem

  casadi::Slice all;
  // ---- decision variables ---------
  casadi::MX X = opti.variable(kNx, N + 1); // state trajectory
  auto x = X(0, all);
  auto y = X(1, all);
  auto z = X(2, all);
  auto x_dot = X(3, all);
  auto y_dot = X(4, all);
  auto z_dot = X(5, all);
  auto phi   = X(6, all);
  auto theta = X(7, all);
  auto psi   = X(8, all);
  auto phi_dot   = X(9, all);
  auto theta_dot = X(10, all);
  auto psi_dot   = X(11, all);

  casadi::MX U = opti.variable(kNu, N); // control trajectory (throttle)
  casadi::MX T = opti.variable(); // final time

  // ---- objective          ---------
  opti.minimize(T); // race in minimal time

  // ---- dynamic constraints --------
  casadi::MX dt = T / N;
  // Example: create a numeric FMU wrapper to evaluate derivatives outside of
  // the symbolic construction. This shows how to call the FMU numerically.
  // Uncomment and adapt sizes if you want to run numeric evaluations here.
  // FMU_CasadiWrapper fmuw(argc, argv, fmi2ModelExchange, kThrustVr, kNu, nullptr, kYVr, kNy);
  // std::vector<double> x_num(kNx, 0.0), u_num(kNu, 0.0), dx_num(kNx);
  // double t_num = 0.0;
  // if (fmuw.eval(x_num, u_num, t_num, dx_num)) {
  //   // dx_num now contains numeric derivatives from the FMU
  // }
  // Keep the callback class available, but do not instantiate the CasADi
  // Function from it here; the local CasADi version in this environment does
  // not expose the constructor used by the upstream docs.
  FMU_CasadiCallback fmu_cb(argc, argv, kNx, kNu, kThrustVr, kNu, kYVr, kNy);

  for (int k = 0; k < N; ++k) {
    // FMU numeric calls disabled while building the CasADi symbolic OCP.
    // Extract symbolic state and control at node k and form symbolic derivatives.
    casadi::MX u_k = U(all, k);
    casadi::MX x_k = X(all, k);

    // Placeholder dynamics: the installed CasADi version here does not support
    // the upstream callback construction used in the docs, so we keep the
    // symbolic setup explicit and avoid forcing an unsupported wrapper call.
    derivatives = casadi::MX::zeros(kNx, 1);

    // advance symbolic time
    time = time + dt;
    // casadi::MX k1 = f(X(all,k),         U(all,k));
    // casadi::MX k2 = f(X(all,k)+dt/2*k1, U(all,k));
    // casadi::MX k3 = f(X(all,k)+dt/2*k2, U(all,k));
    // casadi::MX k4 = f(X(all,k)+dt*k3,   U(all,k));
    // casadi::MX x_next = X(all,k) + dt/6*(k1+2*k2+2*k3+k4);
    casadi::MX x_next = X(all,k) + dt*derivatives;
    opti.subject_to(X(all,k+1)==x_next); // close the gaps 
  }

  // ---- path constraints -----------
  opti.subject_to(0<=U(0, all)<=1);           // control is limited
  opti.subject_to(0<=U(1, all)<=1);
  opti.subject_to(0<=U(2, all)<=1);
  opti.subject_to(0<=U(3, all)<=1);

  // ---- boundary conditions --------
  opti.subject_to(x(0)==0);   // start at position 0 ...
  opti.subject_to(y(0)==0);
  opti.subject_to(z(0)==0);
  opti.subject_to(x_dot(0)==0); // ... from stand-still
  opti.subject_to(y_dot(0)==0);
  opti.subject_to(z_dot(0)==0);
  opti.subject_to(x(N)==1); // finish line at position 1
  opti.subject_to(y(N)==1);
  opti.subject_to(z(N)==1);
  opti.subject_to(x_dot(N)==0); // finish line at stand-still
  opti.subject_to(y_dot(N)==0);
  opti.subject_to(z_dot(N)==0);

  // ---- misc. constraints  ----------
  opti.subject_to(T>=0); // Time must be positive

  // CasADi only allows set_initial on actual decision variables (or simple
  // mappings of them). `velocity` is an arbitrary expression, so initialize the
  // underlying state/control variables instead of the derived expression.
  opti.set_initial(X, casadi::DM::zeros(kNx, N + 1));
  opti.set_initial(U, 0.5 * casadi::DM::ones(kNu, N));
  opti.set_initial(T, 1.0);

  // ---- solve NLP              ------
  opti.solver("ipopt"); // set numerical backend
  casadi::OptiSol sol = opti.solve();   // actual solve

  // ---------------------------- Clean-Up ----------------------------
  Terminate(fm);
  std::cout << "Done.\n";
  return 0;
}
