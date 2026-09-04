#pragma once

#include "FMU_CasadiWrapper.hpp"
#include <casadi/casadi.hpp>
#include <memory>

// A CasADi Callback that wraps an FMU numeric evaluator (FMU_CasadiWrapper).
// Inputs: [x (nx), u (nu)]
// Outputs: [dx (nx)]
class FMU_CasadiCallback : public casadi::Callback {
 public:
  FMU_CasadiCallback(int argc, char** argv, std::size_t nx, std::size_t nu,
                     const fmi2ValueReference* thrustVr = nullptr, std::size_t nThrust = 0,
                     const fmi2ValueReference* bodyVr = nullptr, std::size_t nBody = 0,
                     const std::string& name = "FMU_CasadiCallback");

  // Callback API
  casadi_int get_n_in() const override;
  casadi_int get_n_out() const override;
  std::vector<casadi::Sparsity> get_sparsity_in() const override;
  std::vector<casadi::Sparsity> get_sparsity_out() const override;
  std::vector<casadi::DM> eval(const std::vector<casadi::DM>& args) const override;

 private:
  std::shared_ptr<FMU_CasadiWrapper> m_wrapper;
  std::size_t m_nx;
  std::size_t m_nu;
};
