#include "tasks/auto_aim/hero_solver.hpp"

#include "tasks/auto_aim/solver.hpp"

namespace auto_aim {

void HeroSolver::apply_to_solver(Solver& solver) const { solver.set_R_gimbal2world(q_board_); }

} // namespace auto_aim
