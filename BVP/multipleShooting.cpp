// FMI 2.0 multiple shooting host for the Quadcopter FMU.
// Does not modify model sources. Load the unzipped FMU (or pass its path) and step.

#include "../Toolbox/FMU.h"
#include <casadi/casadi.hpp>

// From modelDescription.xml (Quadcopter FMI 2.0)
static const fmi2ValueReference kThrustVr[4] = {348, 349, 350, 351};
static const fmi2ValueReference kYVr[3] = {3, 4, 5};

int main(int argc, char** argv) {
  // ------------------------------ FMU -------------------------------
  const fmi2Boolean loggingOn = (argc > 2 && std::strcmp(argv[2], "--verbose") == 0) ? fmi2True
                               : (argc > 1 && std::strcmp(argv[1], "--verbose") == 0) ? fmi2True
                                                                                     : fmi2False;

  const fmi2Real tStart = 0.0;
  const fmi2Real* tStop = nullptr;
  FMU fm = Initialise(argc, argv, fmi2ModelExchange, tStart, tStop, loggingOn);

  constexpr std::size_t kNx = 12;
  constexpr std::size_t kNu = 4;
  constexpr std::size_t kNy = 3;
//   fmi2Real states[kNx]{};
  fmi2Real derivatives[kNx]{};
//   check(fm.fmi2GetContinuousStates(fm.c, states, kNx), "fmi2GetContinuousStates");

  fmi2Real Y[kNy]{};
//   check(fm.fmi2GetReal(fm.c, kYVr, kNy, Y), "fmi2GetReal(body.r_0)");

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

  auto velocity = sqrt(x_dot*x_dot + y_dot*y_dot + z_dot*z_dot);

  casadi::MX U = opti.variable(kNu, N); // control trajectory (throttle)
  casadi::MX T = opti.variable(); // final time

  // ---- objective          ---------
  opti.minimize(T); // race in minimal time

  // ---- dynamic constraints --------
  casadi::MX dt = T / N;
  for (int k = 0; k < N; ++k) {
    check(fm.fmi2SetReal(fm.c, kThrustVr, kNu, U(all,k)), "fmi2SetReal(thrust)");

    check(fm.fmi2SetTime(fm.c, time), "fmi2SetTime");
    check(fm.fmi2GetDerivatives(fm.c, derivatives, kNx), "fmi2GetDerivatives");
    check(fm.fmi2SetContinuousStates(fm.c, X(all, k), kNx), "fmi2SetContinuousStates");

    time += dt;
    check(fm.fmi2SetTime(fm.c, time), "fmi2SetTime");

    check(fm.fmi2GetReal(fm.c, kYVr, kNy, Y), "fmi2GetReal(body.r_0)");
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
  opti.subject_to(Y(0,0)==0);   // start at position 0 ...
  opti.subject_to(Y(1,0)==0);
  opti.subject_to(Y(2,0)==0);
  opti.subject_to(velocity(0)==0); // ... from stand-still 
  opti.subject_to(Y(0,N)==1); // finish line at position 1
  opti.subject_to(Y(1,N)==1);
  opti.subject_to(Y(2,N)==1);
  opti.subject_to(velocity(N)==0); // finish line at stand-still

  // ---- misc. constraints  ----------
  opti.subject_to(T>=0); // Time must be positive

  // ---- initial values for solver ---
  opti.set_initial(velocity, 1);
  opti.set_initial(T, 1);

  // ---- solve NLP              ------
  opti.solver("ipopt"); // set numerical backend
  casadi::OptiSol sol = opti.solve();   // actual solve

  // ---------------------------- Clean-Up ----------------------------
  Terminate(fm);
  std::cout << "Done.\n";
  return 0;
}
