#pragma once

#include "search.hpp"

#include <functional>
#include <iosfwd>

namespace routing {

using IncumbentSink = std::function<void(const Plan&, const Evaluation&, double elapsed_ms)>;

struct OperatorEvent {
    int iteration;
    int destroy, repair, noise;
    int requested, removed;
    bool completed, accepted, improved_current, improved_best;
    double destroy_us, repair_us;
};

using OperatorSink = std::function<void(const OperatorEvent&)>;
using NeighborhoodSink = std::function<void(int iteration, const Plan&, const std::vector<int>&)>;

Instance read_instance(std::istream& input);
Settings read_settings(std::istream& input, int customer_count);
Plan make_initial_plan(const Instance& instance, Clock::time_point deadline);

SearchResult solve_alns(const Instance& instance, const Settings& settings, bool allow_cross_stop,
                        const IncumbentSink& incumbent, const OperatorSink& operators,
                        const NeighborhoodSink& neighborhood);
SearchResult solve_order_first(const Instance& instance, const Settings& settings, int block_limit,
                               int perturb_moves, const IncumbentSink& incumbent);
SearchResult solve_vns(const Instance& instance, const Settings& settings, int shake_moves,
                       const IncumbentSink& incumbent);

}  // namespace routing
