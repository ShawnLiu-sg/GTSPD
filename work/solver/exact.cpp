#include "routing.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace routing {
namespace {

using Mask = unsigned int;
constexpr std::size_t no_index = std::numeric_limits<std::size_t>::max();
constexpr double infinity = std::numeric_limits<double>::infinity();

struct Suffix {
    int first;
    std::size_t next;
    double distance;
    double energy;
    double mass;
};

struct Pattern {
    Mask served;
    std::size_t suffix;
    double distance;
    double energy;
    double duration;
    double cost;
};

struct ThresholdChoice {
    double value = infinity;
    double threshold = 0.0;
};

struct Cell {
    double value = infinity;
    std::size_t pattern = no_index;
};

struct StopProblem {
    std::vector<Suffix> labels;
    std::vector<Pattern> patterns;
    std::vector<ThresholdChoice> best;
};

template <class Label>
void insert_nondominated(std::vector<Label>& frontier, const Label& candidate) {
    const auto dominates = [](const Label& lhs, const Label& rhs) {
        return lhs.distance <= rhs.distance && lhs.energy <= rhs.energy;
    };
    if (std::any_of(frontier.begin(), frontier.end(), [&](const Label& existing) {
            return dominates(existing, candidate);
        })) {
        return;
    }
    frontier.erase(std::remove_if(frontier.begin(), frontier.end(), [&](const Label& existing) {
        return dominates(candidate, existing);
    }), frontier.end());
    frontier.push_back(candidate);
}

bool generate_patterns(const Instance& instance, int anchor, const std::vector<int>& pool,
                       double truck_prefix, Clock::time_point deadline, StopProblem& problem) {
    if (!instance.customers[anchor].launch_allowed) {
        return true;
    }

    const Mask state_count = Mask{1} << pool.size();
    std::vector<std::vector<Suffix>> frontier(state_count * pool.size());
    const auto suffixes = [&](Mask mask, std::size_t first) -> std::vector<Suffix>& {
        return frontier[mask * pool.size() + first];
    };
    const double truck_payload_price = instance.carbon_cost_per_kg * instance.fuel_co2_per_l
                                      * instance.truck_payload_l_per_s_kg * truck_prefix
                                      / instance.truck_speed;
    const double electricity_price = instance.carbon_cost_per_kg * instance.electricity_co2_per_kwh;

    for (std::size_t i = 0; i < pool.size(); ++i) {
        const int customer = pool[i];
        const double mass = instance.customers[customer].mass;
        const double distance = instance.air[customer][anchor];
        const double energy = flight_leg_energy(instance, distance, 0.0);
        if (instance.customers[customer].drone_eligible && mass <= instance.drone_capacity &&
            energy <= instance.drone_budget) {
            suffixes(Mask{1} << i, i).push_back({customer, no_index, distance, energy, mass});
        }
    }

    for (Mask mask = 1; mask < state_count; ++mask) {
        if (Clock::now() >= deadline) {
            return false;
        }
        std::vector<Pattern> completed;
        for (std::size_t first = 0; first < pool.size(); ++first) {
            // Every predecessor mask is smaller, so this state's frontier is now final.
            for (const Suffix& suffix : suffixes(mask, first)) {
                if (Clock::now() >= deadline) {
                    return false;
                }
                const std::size_t label_id = problem.labels.size();
                problem.labels.push_back(suffix);
                const double launch_distance = instance.air[anchor][suffix.first];
                const double total_distance = suffix.distance + launch_distance;
                const double total_energy = suffix.energy
                    + flight_leg_energy(instance, launch_distance, suffix.mass);
                if (total_energy <= instance.drone_budget) {
                    const double cost = instance.drone_cost_per_m * total_distance
                                        + electricity_price * total_energy
                                        + truck_payload_price * suffix.mass;
                    insert_nondominated(completed, Pattern{mask, label_id, total_distance,
                        total_energy, total_distance / instance.drone_speed, cost});
                }

                for (std::size_t i = 0; i < pool.size(); ++i) {
                    const Mask bit = Mask{1} << i;
                    const int customer = pool[i];
                    if ((mask & bit) || !instance.customers[customer].drone_eligible) {
                        continue;
                    }
                    const double mass = suffix.mass + instance.customers[customer].mass;
                    const double distance = instance.air[customer][suffix.first];
                    const double energy = suffix.energy
                        + flight_leg_energy(instance, distance, suffix.mass);
                    if (mass <= instance.drone_capacity && energy <= instance.drone_budget) {
                        insert_nondominated(suffixes(mask | bit, i), Suffix{customer, label_id,
                            suffix.distance + distance, energy, mass});
                    }
                }
            }
        }
        problem.patterns.insert(problem.patterns.end(), completed.begin(), completed.end());
    }

    std::stable_sort(problem.patterns.begin(), problem.patterns.end(),
        [](const Pattern& lhs, const Pattern& rhs) { return lhs.duration < rhs.duration; });
    return true;
}

void enable_pattern(const std::vector<Pattern>& patterns, std::size_t index,
                    std::vector<std::size_t>& cheapest) {
    const Pattern& pattern = patterns[index];
    std::size_t& current = cheapest[pattern.served];
    if (current == no_index || pattern.cost < patterns[current].cost) {
        current = index;
    }
}

bool solve_threshold(const std::vector<Pattern>& patterns, const std::vector<std::size_t>& cheapest,
                     int slots, Clock::time_point deadline, std::vector<Cell>& table) {
    const Mask state_count = static_cast<Mask>(cheapest.size());
    table[0] = {0.0, no_index};
    for (int k = 1; k <= slots; ++k) {
        Cell* row = table.data() + k * state_count;
        const Cell* previous = row - state_count;
        row[0] = {0.0, no_index};
        for (Mask mask = 1; mask < state_count; ++mask) {
            if (Clock::now() >= deadline) {
                return false;
            }
            Cell best;
            const Mask pivot = mask & (Mask{0} - mask);
            const Mask remainder = mask ^ pivot;
            for (Mask subset = remainder;; subset = (subset - 1) & remainder) {
                const std::size_t index = cheapest[subset | pivot];
                if (index != no_index) {
                    const double value = patterns[index].cost + previous[remainder ^ subset].value;
                    if (value < best.value) {
                        best = {value, index};
                    }
                }
                if (subset == 0) {
                    break;
                }
            }
            row[mask] = best;
        }
    }
    return true;
}

bool solve_stop(const Instance& instance, int slots, Mask state_count, Clock::time_point deadline,
                StopProblem& problem) {
    problem.best.resize(state_count);
    problem.best[0] = {0.0, 0.0};
    std::vector<std::size_t> cheapest(state_count, no_index);
    std::vector<Cell> table((slots + 1) * state_count);
    for (std::size_t next = 0; next < problem.patterns.size();) {
        const double threshold = problem.patterns[next].duration;
        do {
            enable_pattern(problem.patterns, next++, cheapest);
        } while (next < problem.patterns.size() && problem.patterns[next].duration == threshold);

        if (!solve_threshold(problem.patterns, cheapest, slots, deadline, table)) {
            return false;
        }
        const double dwell_cost = instance.wait_cost_per_s * threshold;
        for (Mask mask = 1; mask < state_count; ++mask) {
            const double value = table[slots * state_count + mask].value + dwell_cost;
            if (value < problem.best[mask].value) {
                problem.best[mask] = {value, threshold};
            }
        }
    }
    return true;
}

bool reconstruct(const StopProblem& problem, int slots, Mask state_count, Mask served,
                 Clock::time_point deadline, std::vector<Sortie>& sorties) {
    sorties.clear();
    if (served == 0) {
        return true;
    }
    const double threshold = problem.best[served].threshold;
    std::vector<std::size_t> cheapest(state_count, no_index);
    for (std::size_t index = 0;
         index < problem.patterns.size() && problem.patterns[index].duration <= threshold; ++index) {
        enable_pattern(problem.patterns, index, cheapest);
    }
    std::vector<Cell> table((slots + 1) * state_count);
    if (!solve_threshold(problem.patterns, cheapest, slots, deadline, table)) {
        return false;
    }
    for (int k = slots; served != 0; --k) {
        const Pattern& pattern = problem.patterns[table[k * state_count + served].pattern];
        Sortie sortie;
        for (std::size_t label = pattern.suffix; label != no_index; label = problem.labels[label].next) {
            sortie.push_back(problem.labels[label].first);
        }
        sorties.push_back(std::move(sortie));
        served ^= pattern.served;
    }
    return true;
}

}  // namespace

bool improve_exact(const Instance& instance, Plan& plan, const std::vector<int>& stop_indices,
                   Clock::time_point deadline) {
    std::vector<int> pool;
    for (int index : stop_indices) {
        for (const Sortie& sortie : plan[index].sorties) {
            pool.insert(pool.end(), sortie.begin(), sortie.end());
        }
    }
    if (pool.empty()) {
        return true;
    }
    std::sort(pool.begin(), pool.end());

    const int slots = std::min(instance.drones, static_cast<int>(pool.size()));
    const Mask state_count = Mask{1} << pool.size();
    const Mask full = state_count - 1;
    std::vector<double> truck_prefixes(plan.size());
    int previous = 0;
    double prefix = 0.0;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        prefix += instance.road[previous][plan[i].node];
        truck_prefixes[i] = prefix;
        previous = plan[i].node;
    }

    std::vector<StopProblem> stops(stop_indices.size());
    for (std::size_t stop = 0; stop < stop_indices.size(); ++stop) {
        const int index = stop_indices[stop];
        if (!generate_patterns(instance, plan[index].node, pool, truck_prefixes[index], deadline,
                               stops[stop]) ||
            !solve_stop(instance, slots, state_count, deadline, stops[stop])) {
            return false;
        }
    }

    std::vector<Mask> assignment(1, full);
    if (stops.size() == 2) {
        Mask best_subset = 0;
        double best = infinity;
        for (Mask subset = 0; subset < state_count; ++subset) {
            const double value = stops[0].best[subset].value + stops[1].best[full ^ subset].value;
            if (value < best) {
                best = value;
                best_subset = subset;
            }
        }
        assignment = {best_subset, full ^ best_subset};
    }

    Plan candidate = plan;
    for (std::size_t stop = 0; stop < stops.size(); ++stop) {
        if (!reconstruct(stops[stop], slots, state_count, assignment[stop], deadline,
                         candidate[stop_indices[stop]].sorties)) {
            return false;
        }
    }
    const Evaluation candidate_value = evaluate(instance, candidate);
    const Evaluation incumbent_value = evaluate(instance, plan);
    if (Clock::now() >= deadline) {
        return false;
    }
    if (candidate_value.feasible && candidate_value.total < incumbent_value.total) {
        plan = std::move(candidate);
    }
    return true;
}

}  // namespace routing
