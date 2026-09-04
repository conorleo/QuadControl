#include "FMU_CasadiCallback.hpp"
#include <algorithm>

FMU_CasadiCallback::FMU_CasadiCallback(int argc, char** argv, std::size_t nx, std::size_t nu,
                                       const fmi2ValueReference* thrustVr, std::size_t nThrust,
                                       const fmi2ValueReference* bodyVr, std::size_t nBody,
                                       const std::string& name)
    : casadi::Callback(name), m_wrapper(std::make_shared<FMU_CasadiWrapper>(argc, argv, fmi2ModelExchange,
                                                                             thrustVr, nThrust, nullptr,
                                                                             bodyVr, nBody)),
      m_nx(nx), m_nu(nu) {}

casadi_int FMU_CasadiCallback::get_n_in() const { return 2; }
casadi_int FMU_CasadiCallback::get_n_out() const { return 1; }

std::vector<casadi::Sparsity> FMU_CasadiCallback::get_sparsity_in() const {
  return {casadi::Sparsity::dense(m_nx, 1), casadi::Sparsity::dense(m_nu, 1)};
}

std::vector<casadi::Sparsity> FMU_CasadiCallback::get_sparsity_out() const {
  return {casadi::Sparsity::dense(m_nx, 1)};
}

std::vector<casadi::DM> FMU_CasadiCallback::eval(const std::vector<casadi::DM>& args) const {
  // args[0] : x (nx) as DM column
  // args[1] : u (nu) as DM column
  std::vector<double> x(m_nx), u(m_nu), dx(m_nx);

  // Convert DM -> std::vector<double>
  auto to_std = [](const casadi::DM& d, std::vector<double>& out) {
    std::size_t n = out.size();
    casadi::DM dd = casadi::DM::reshape(d, n, 1);
    for (std::size_t i = 0; i < n; ++i) {
      out[i] = static_cast<double>(dd(i));
    }
  };

  to_std(args.at(0), x);
  to_std(args.at(1), u);

  double t0 = 0.0; // callback does not change FMU time; integrator manages time externally
  bool ok = m_wrapper->eval(x, u, t0, dx);
  if (!ok) {
    throw std::runtime_error("FMU callback eval failed");
  }

  casadi::DM out = casadi::DM::zeros(m_nx, 1);
  for (std::size_t i = 0; i < m_nx; ++i) out(i) = dx[i];

  return {out};
}
