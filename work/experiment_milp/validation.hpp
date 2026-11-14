#pragma once

#include "../compare/benchmark.hpp"
#include <cstddef>
#include <limits>
#include <string>

namespace validation {

using namespace routing;
constexpr double infinity = std::numeric_limits<double>::infinity();

struct CutSelection {
    bool packing{}, symmetry{}, cover{}, energy{};
};

struct MilpResult {
    int status{}, incumbents{}, variables{}, constraints{}, packing_bound{};
    double objective = infinity, lower_bound = -infinity, root_bound = -infinity;
    double solver_s{}, elapsed_s{}, nodes{}, decoded_cost = infinity, decoded_wait_s{};
    int relaxation_status{};
    double relaxation_objective = infinity, relaxation_s{};
    bool feasible{};
    Plan plan;
};

struct OracleResult {
    Evaluation checked;
    Plan plan;
    std::size_t truck_tours{}, complete_assignments{};
};

struct RelaxationResult {
    int status{}, variables{}, constraints{}, packing_bound{};
    double objective = infinity, elapsed_s{};
};

Evaluation evaluate_forward(const Instance& instance, const Plan& plan);
Flight evaluate_flight_forward(const Instance& instance, int anchor, const Sortie& route);
OracleResult exhaustive(const Instance& instance);
MilpResult solve_milp(const Instance& instance, const Plan& start, CutSelection cuts, double seconds,
                      bool warm_start);
RelaxationResult solve_relaxation(const Instance& instance, CutSelection cuts, double seconds);

struct LocalPattern {
    Sortie route;
    unsigned customers;
    double dwell, cost;
};

MilpResult solve_local_milp(const Instance& instance, const Plan& start, const std::vector<int>& stops,
                           const std::vector<std::vector<LocalPattern>>& patterns,
                           unsigned pool, double seconds);

}  // namespace validation
