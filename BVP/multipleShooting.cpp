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

  const fmi2Real tStart = 0.0;
  const fmi2Real* tStop = nullptr;
  FMU fm = Initialise(argc, argv, fmi2ModelExchange, tStart, tStop, loggingOn);

  constexpr std::size_t kNx = 12;
  constexpr std::size_t kNu = 4;
  constexpr std::size_t kNy = 3;
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

  auto velocity = sqrt(x_dot*x_dot + y_dot*y_dot + z_dot*z_dot);

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
  // Create a CasADi callback that wraps the FMU so we can use it symbolically.
  auto fmu_cb = std::make_shared<FMU_CasadiCallback>(argc, argv, kNx, kNu, kThrustVr, kNu, kYVr, kNy);
  casadi::Function fmu_func(fmu_cb);

  for (int k = 0; k < N; ++k) {
    // FMU numeric calls disabled while building the CasADi symbolic OCP.
    // Extract symbolic state and control at node k and form symbolic derivatives.
    casadi::MX u_k = U(all, k);
    casadi::MX x_k = X(all, k);

    // Evaluate FMU dynamics symbolically via the CasADi callback (numeric
    // FMU calls happen during solve, CasADi will finite-difference if needed).
    std::vector<casadi::MX> args{x_k, u_k};
    std::vector<casadi::MX> fmu_out = fmu_func(args);
    derivatives = fmu_out[0];

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
  opti.subject_to(velocity(0)==0); // ... from stand-still 
  opti.subject_to(x(N)==1); // finish line at position 1
  opti.subject_to(y(N)==1);
  opti.subject_to(z(N)==1);
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
