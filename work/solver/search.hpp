#pragma once

#include "routing.hpp"

#include <array>
#include <cstdint>

namespace routing {

struct Settings {
    std::uint64_t seed{};
    int iterations{}, wall_ms{};
    int q_min{}, q_max{};
    double sigmoid_alpha{}, sigmoid_beta{};
    double initial_temperature{}, cooling{}, min_temperature{};
    int update_every{};
    double decay{}, noise{};
    std::array<double, 4> rewards{};
    int exact_every{}, exact_pool{}, exact_ms{};
};

struct SearchResult {
    Plan best;
    Evaluation evaluation;
    double initial_cost{}, elapsed_s{};
    int iterations{}, exact_calls{}, exact_complete{}, exact_improvements{};
};

SearchResult solve(const Instance& instance, const Settings& settings);

}  // namespace routing
