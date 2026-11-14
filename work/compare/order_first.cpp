#include "benchmark.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <random>
#include <utility>
#include <vector>

namespace routing {
namespace {

using Mask = unsigned int;
constexpr double infinity = std::numeric_limits<double>::infinity();

struct BlockChoice {
    double cost = infinity;
    std::vector<Sortie> sorties;
};

struct Pattern {
    Sortie route;
    double cost;
    double duration;
};

struct SplitLabel {
    double cost = infinity;
    int block_start = 0;
    int previous_anchor = 0;
    std::vector<Sortie> sorties;
};

double road_cost(const Instance& instance, int from, int to, double load) {
    const double distance = instance.road[from][to];
    const double fuel = (instance.truck_empty_l_per_s + instance.truck_payload_l_per_s_kg * load)
                        * distance / instance.truck_speed;
    return distance * instance.truck_cost_per_m
           + fuel * instance.fuel_co2_per_l * instance.carbon_cost_per_kg;
}

void insert_pattern(std::vector<Pattern>& front, Pattern pattern) {
    const auto dominates = [](const Pattern& a, const Pattern& b) {
        return a.cost <= b.cost && a.duration <= b.duration;
    };
    if (std::any_of(front.begin(), front.end(), [&](const Pattern& other) {
            return dominates(other, pattern);
        })) {
        return;
    }
    front.erase(std::remove_if(front.begin(), front.end(), [&](const Pattern& other) {
        return dominates(pattern, other);
    }), front.end());
    front.push_back(std::move(pattern));
}

bool evaluate_block(const Instance& instance, const std::vector<int>& order, int begin, int end,
                    int anchor_position, Clock::time_point deadline, BlockChoice& result) {
    if (end == begin + 1) {
        result.cost = 0.0;
        return true;
    }
    const int anchor = order[anchor_position];
    if (instance.drones == 0 || !instance.customers[anchor].launch_allowed) {
        return true;
    }

    std::vector<int> air_customers;
    for (int i = begin; i < end; ++i) {
        if (i == anchor_position) {
            continue;
        }
        const int node = order[i];
        if (!instance.customers[node].drone_eligible) {
            return true;
        }
        air_customers.push_back(node);
    }
    std::sort(air_customers.begin(), air_customers.end());
    const Mask full = (Mask{1} << air_customers.size()) - 1;
    std::vector<std::vector<Pattern>> patterns(full + 1);
    const double electricity_cost = instance.carbon_cost_per_kg * instance.electricity_co2_per_kwh;
    for (Mask served = 1; served <= full; ++served) {
        if (Clock::now() >= deadline) {
            return false;
        }
        Sortie route;
        for (std::size_t i = 0; i < air_customers.size(); ++i) {
            if (served & (Mask{1} << i)) {
                route.push_back(air_customers[i]);
            }
        }
        do {
            if (Clock::now() >= deadline) {
                return false;
            }
            const Flight flight = evaluate_sortie(instance, anchor, route);
            if (flight.feasible) {
                const double cost = instance.drone_cost_per_m * flight.distance
                                    + electricity_cost * flight.energy;
                insert_pattern(patterns[served], Pattern{route, cost, flight.duration});
            }
        } while (std::next_permutation(route.begin(), route.end()));
    }

    std::vector<Sortie> chosen;
    const auto partition = [&](auto&& visit, Mask remaining, int slots, double flight_cost,
                               double dwell) -> bool {
        if (Clock::now() >= deadline) {
            return false;
        }
        const double bound = flight_cost + instance.wait_cost_per_s * dwell;
        if (bound >= result.cost) {
            return true;
        }
        if (remaining == 0) {
            result = {bound, chosen};
            return true;
        }
        if (slots == 0) {
            return true;
        }
        const Mask pivot = remaining & (Mask{0} - remaining);
        const Mask rest = remaining ^ pivot;
        for (Mask subset = rest;; subset = (subset - 1) & rest) {
            if (Clock::now() >= deadline) {
                return false;
            }
            for (const Pattern& pattern : patterns[subset | pivot]) {
                chosen.push_back(pattern.route);
                const bool complete = visit(visit, rest ^ subset, slots - 1,
                                            flight_cost + pattern.cost,
                                            std::max(dwell, pattern.duration));
                chosen.pop_back();
                if (!complete) {
                    return false;
                }
            }
            if (subset == 0) {
                break;
            }
        }
        return true;
    };
    return partition(partition, full, instance.drones, 0.0, 0.0);
}

bool split_order(const Instance& instance, const std::vector<int>& order, int block_limit,
                 Clock::time_point deadline, Plan& plan) {
    const int count = static_cast<int>(order.size());
    std::vector<double> remaining_mass(count + 1, 0.0);
    for (int i = count - 1; i >= 0; --i) {
        remaining_mass[i] = remaining_mass[i + 1] + instance.customers[order[i]].mass;
    }
    std::vector<std::vector<SplitLabel>> labels(count + 1, std::vector<SplitLabel>(count));

    for (int begin = 0; begin < count; ++begin) {
        const int furthest_end = std::min(count, begin + block_limit);
        for (int anchor = begin; anchor < furthest_end; ++anchor) {
            if (Clock::now() >= deadline) {
                return false;
            }
            double incoming_cost = infinity;
            int previous_anchor = -1;
            if (begin == 0) {
                incoming_cost = road_cost(instance, 0, order[anchor], remaining_mass[0]);
            } else {
                for (int previous = std::max(0, begin - block_limit); previous < begin; ++previous) {
                    const double total = labels[begin][previous].cost
                        + road_cost(instance, order[previous], order[anchor], remaining_mass[begin]);
                    if (total < incoming_cost) {
                        incoming_cost = total;
                        previous_anchor = previous;
                    }
                }
            }

            for (int end = anchor + 1; end <= furthest_end; ++end) {
                SplitLabel& label = labels[end][anchor];
                if (incoming_cost >= label.cost) {
                    continue;
                }
                BlockChoice block;
                if (!evaluate_block(instance, order, begin, end, anchor, deadline, block)) {
                    return false;
                }
                const double total = incoming_cost + block.cost;
                if (total < label.cost) {
                    label = {total, begin, previous_anchor, std::move(block.sorties)};
                }
            }
        }
    }

    int anchor = -1;
    double best = infinity;
    for (int last = std::max(0, count - block_limit); last < count; ++last) {
        const double total = labels[count][last].cost + road_cost(instance, order[last], 0, 0.0);
        if (total < best) {
            best = total;
            anchor = last;
        }
    }
    int end = count;
    while (end != 0) {
        SplitLabel& label = labels[end][anchor];
        plan.push_back({order[anchor], std::move(label.sorties)});
        anchor = label.previous_anchor;
        end = label.block_start;
    }
    std::reverse(plan.begin(), plan.end());
    return true;
}

void perturb_order(std::vector<int>& order, int moves, std::mt19937_64& random) {
    const int count = static_cast<int>(order.size());
    for (int move = 0; move < moves; ++move) {
        const int first = std::uniform_int_distribution<int>(0, count - 1)(random);
        int second = std::uniform_int_distribution<int>(0, count - 2)(random);
        if (second >= first) {
            ++second;
        }
        const auto [left, right] = std::minmax(first, second);
        std::reverse(order.begin() + left, order.begin() + right + 1);
    }
}

}  // namespace

SearchResult solve_order_first(const Instance& instance, const Settings& settings, int block_limit,
                               int perturb_moves, const IncumbentSink& incumbent) {
    const auto start = Clock::now();
    const auto deadline = start + std::chrono::milliseconds(settings.wall_ms);
    SearchResult result;
    result.best = make_initial_plan(instance, deadline);
    result.evaluation = evaluate(instance, result.best);
    result.initial_cost = result.evaluation.total;

    const auto record = [&](Clock::time_point time) {
        if (incumbent && time < deadline) {
            const double elapsed = std::chrono::duration<double, std::milli>(time - start).count();
            incumbent(result.best, result.evaluation, elapsed);
        }
    };
    record(Clock::now());

    std::vector<int> best_order;
    for (const Stop& stop : result.best) {
        best_order.push_back(stop.node);
    }
    std::vector<int> order = best_order;
    std::mt19937_64 random(settings.seed);
    while (Clock::now() < deadline) {
        Plan candidate;
        if (!split_order(instance, order, block_limit, deadline, candidate)) {
            break;
        }
        const Evaluation evaluation = evaluate(instance, candidate);
        const auto finished = Clock::now();
        if (finished >= deadline) {
            break;
        }
        ++result.iterations;
        if (evaluation.feasible && evaluation.total < result.evaluation.total) {
            result.best = std::move(candidate);
            result.evaluation = evaluation;
            best_order = order;
            record(finished);
        }
        if (perturb_moves == 0 || best_order.size() < 2) {
            break;
        }
        order = best_order;
        perturb_order(order, perturb_moves, random);
    }

    result.elapsed_s = std::chrono::duration<double>(Clock::now() - start).count();
    return result;
}

}  // namespace routing
