#include "benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <tuple>
#include <utility>

namespace routing {
namespace {

using Random = std::mt19937_64;

template<std::size_t N>
struct Operators {
    std::array<double, N> weights;
    std::array<double, N> scores{};
    std::array<int, N> calls{};

    Operators() { weights.fill(1.0); }

    int select(Random& rng) const {
        return std::discrete_distribution<int>(weights.begin(), weights.end())(rng);
    }

    void record(int choice, double reward) {
        ++calls[choice];
        scores[choice] += reward;
    }

    void update(double decay) {
        for (std::size_t i = 0; i < N; ++i) {
            if (calls[i] > 0) {
                const double average = scores[i] / calls[i];
                weights[i] = (1.0 - decay) * weights[i] + decay * average;
            }
        }
        calls.fill(0);
        scores.fill(0.0);
    }
};

std::vector<int> customer_ids(const Instance& instance) {
    std::vector<int> ids(instance.customers.size() - 1);
    std::iota(ids.begin(), ids.end(), 1);
    return ids;
}

Plan initial_plan(const Instance& instance, Clock::time_point deadline) {
    auto remaining = customer_ids(instance);
    Plan plan;
    int previous = 0;
    while (!remaining.empty()) {
        const auto next = std::min_element(remaining.begin(), remaining.end(), [&](int a, int b) {
            return std::tie(instance.road[previous][a], a) < std::tie(instance.road[previous][b], b);
        });
        previous = *next;
        plan.push_back({previous, {}});
        remaining.erase(next);
    }

    double cost = evaluate(instance, plan).total;
    while (Clock::now() < deadline) {
        Plan best = plan;
        double best_cost = cost;
        for (std::size_t i = 0; i + 1 < plan.size() && Clock::now() < deadline; ++i) {
            for (std::size_t j = i + 1; j < plan.size() && Clock::now() < deadline; ++j) {
                std::reverse(plan.begin() + i, plan.begin() + j + 1);
                const double trial = evaluate(instance, plan).total;
                if (trial < best_cost) {
                    best = plan;
                    best_cost = trial;
                }
                std::reverse(plan.begin() + i, plan.begin() + j + 1);
            }
        }
        if (best_cost >= cost) break;
        cost = best_cost;
        plan = std::move(best);
    }
    return plan;
}

std::vector<int> remove_customer(Plan& plan, int node) {
    for (auto stop = plan.begin(); stop != plan.end(); ++stop) {
        if (stop->node == node) {
            std::vector<int> removed{node};
            for (const auto& sortie : stop->sorties) {
                removed.insert(removed.end(), sortie.begin(), sortie.end());
            }
            plan.erase(stop);
            return removed;
        }
        for (auto sortie = stop->sorties.begin(); sortie != stop->sorties.end(); ++sortie) {
            const auto found = std::find(sortie->begin(), sortie->end(), node);
            if (found == sortie->end()) continue;
            sortie->erase(found);
            if (sortie->empty()) stop->sorties.erase(sortie);
            return {node};
        }
    }
    return {};
}

struct Edge {
    int from, to;
    double cost;
};

std::vector<int> edge_priority(const Instance& in, const Plan& plan, Random& rng) {
    std::vector<Edge> edges;
    double load = 0;
    for (const auto& customer : in.customers) load += customer.mass;
    int last = 0;
    auto truck_edge = [&](int from, int to) {
        const double distance = in.road[from][to];
        const double fuel = (in.truck_empty_l_per_s + in.truck_payload_l_per_s_kg * load)
                          * distance / in.truck_speed;
        edges.push_back({from, to, in.truck_cost_per_m * distance + in.carbon_cost_per_kg * in.fuel_co2_per_l * fuel});
    };
    for (const auto& stop : plan) {
        truck_edge(last, stop.node);
        load -= in.customers[stop.node].mass;
        for (const auto& sortie : stop.sorties) {
            double air_load = 0;
            for (int id : sortie) air_load += in.customers[id].mass;
            load -= air_load;
            int previous = stop.node;
            auto air_edge = [&](int next) {
                const double distance = in.air[previous][next];
                const double energy = flight_leg_energy(in, distance, air_load);
                const double cost = in.drone_cost_per_m * distance + in.carbon_cost_per_kg * in.electricity_co2_per_kwh * energy;
                edges.push_back({previous, next, cost});
                previous = next;
            };
            for (int id : sortie) {
                air_edge(id);
                air_load -= in.customers[id].mass;
            }
            air_load = 0;
            air_edge(stop.node);
        }
        last = stop.node;
    }
    load = 0;
    truck_edge(last, 0);
    std::vector<double> weights;
    for (const auto& edge : edges) weights.push_back(edge.cost);
    if (std::none_of(weights.begin(), weights.end(), [](double w) { return w > 0; })) return {};
    const auto& edge = edges[std::discrete_distribution<std::size_t>(weights.begin(), weights.end())(rng)];
    std::vector<int> order;
    if (edge.from != 0) order.push_back(edge.from);
    if (edge.to != 0) order.push_back(edge.to);
    return order;
}

std::pair<double, const Sortie*> longest_flight(const Instance& in, const Stop& stop) {
    double duration = 0;
    const Sortie* longest = nullptr;
    for (const auto& sortie : stop.sorties) {
        const double flight = evaluate_sortie(in, stop.node, sortie).duration;
        if (!longest || flight > duration) {
            duration = flight;
            longest = &sortie;
        }
    }
    return {duration, longest};
}

std::vector<int> destroy_order(const Instance& in, const Plan& plan, int op, Random& rng,
                               Clock::time_point deadline) {
    auto fallback = customer_ids(in);
    std::shuffle(fallback.begin(), fallback.end(), rng);
    if (op == 0) return fallback;

    std::vector<int> order;
    if (op == 1) {
        const int seed = fallback.front();
        auto related = customer_ids(in);
        std::sort(related.begin(), related.end(), [&](int a, int b) {
            return std::tie(in.air[seed][a], a) < std::tie(in.air[seed][b], b);
        });
        related.erase(std::find(related.begin(), related.end(), seed));
        related.insert(related.begin(), seed);
        return related;
    }
    if (op == 2 || op == 3) {
        std::vector<bool> launch(in.customers.size(), false);
        for (const auto& stop : plan) launch[stop.node] = !stop.sorties.empty();
        std::vector<std::pair<double, int>> ranked;
        for (int id : customer_ids(in)) {
            if (Clock::now() >= deadline) break;
            if (launch[id] != (op == 3)) continue;
            Plan partial = plan;
            remove_customer(partial, id);
            const auto evaluation = evaluate(in, partial, false);
            if (evaluation.feasible) ranked.emplace_back(evaluation.total, id);
        }
        std::sort(ranked.begin(), ranked.end());
        for (const auto& entry : ranked) order.push_back(entry.second);
    } else if (op == 4) {
        order = edge_priority(in, plan, rng);
    } else {
        std::vector<double> waits;
        std::vector<const Sortie*> sorties;
        for (const auto& stop : plan) {
            const auto [duration, sortie] = longest_flight(in, stop);
            waits.push_back(duration);
            sorties.push_back(sortie);
        }
        if (std::any_of(waits.begin(), waits.end(), [](double wait) { return wait > 0; })) {
            const auto selected = std::discrete_distribution<std::size_t>(waits.begin(), waits.end())(rng);
            order = *sorties[selected];
        }
    }
    order.insert(order.end(), fallback.begin(), fallback.end());
    return order;
}

std::vector<int> destroy(const Instance& in, Plan& plan, int op, int count, Random& rng,
                         Clock::time_point deadline) {
    std::vector<int> removed;
    for (int id : destroy_order(in, plan, op, rng, deadline)) {
        if (static_cast<int>(removed.size()) == count || Clock::now() >= deadline) break;
        Plan trial = plan;
        const auto group = remove_customer(trial, id);
        if (group.empty() || static_cast<int>(removed.size() + group.size()) > count) continue;
        if (!evaluate(in, trial, false).feasible) continue;
        removed.insert(removed.end(), group.begin(), group.end());
        plan = std::move(trial);
    }
    std::sort(removed.begin(), removed.end());
    return removed;
}

enum class Kind { truck, insert_sortie, singleton };

struct Placement {
    int customer;
    Kind kind;
    std::size_t stop, sortie, position;
    double rank{};
};

bool placement_less(const Placement& a, const Placement& b) {
    return std::tie(a.rank, a.customer, a.kind, a.stop, a.sortie, a.position)
         < std::tie(b.rank, b.customer, b.kind, b.stop, b.sortie, b.position);
}

void insert(Plan& plan, const Placement& p) {
    if (p.kind == Kind::truck) {
        plan.insert(plan.begin() + p.stop, Stop{p.customer, {}});
    } else if (p.kind == Kind::singleton) {
        plan[p.stop].sorties.push_back({p.customer});
    } else {
        auto& sortie = plan[p.stop].sorties[p.sortie];
        sortie.insert(sortie.begin() + p.position, p.customer);
    }
}

std::vector<Placement> enumerate_placements(const Instance& in, const Plan& plan, int node,
                                             double current_cost, Clock::time_point deadline) {
    std::vector<Placement> placements;
    auto consider = [&](Placement p) {
        if (Clock::now() >= deadline) return;
        Plan trial = plan;
        insert(trial, p);
        const auto evaluation = evaluate(in, trial, false);
        if (evaluation.feasible) {
            p.rank = evaluation.total - current_cost;
            placements.push_back(p);
        }
    };
    for (std::size_t i = 0; i <= plan.size() && Clock::now() < deadline; ++i) {
        consider({node, Kind::truck, i, 0, 0});
    }
    if (!in.customers[node].drone_eligible || in.customers[node].mass > in.drone_capacity) return placements;
    for (std::size_t i = 0; i < plan.size() && Clock::now() < deadline; ++i) {
        const auto& stop = plan[i];
        if (!in.customers[stop.node].launch_allowed) continue;
        for (std::size_t s = 0; s < stop.sorties.size(); ++s) {
            for (std::size_t pos = 0; pos <= stop.sorties[s].size(); ++pos) {
                consider({node, Kind::insert_sortie, i, s, pos});
            }
        }
        if (static_cast<int>(stop.sorties.size()) < in.drones) consider({node, Kind::singleton, i, 0, 0});
    }
    return placements;
}

bool repair(const Instance& in, Plan& plan, std::vector<int> pending, int op, double noise,
            Random& rng, Clock::time_point deadline) {
    while (!pending.empty()) {
        const double current_cost = evaluate(in, plan, false).total;
        std::vector<std::vector<Placement>> rows;
        double scale = 0;
        for (int node : pending) {
            auto row = enumerate_placements(in, plan, node, current_cost, deadline);
            if (Clock::now() >= deadline) return false;
            for (const auto& p : row) scale = std::max(scale, std::abs(p.rank));
            rows.push_back(std::move(row));
        }
        std::uniform_real_distribution<double> perturb(-noise * scale, noise * scale);
        for (auto& row : rows) {
            if (noise > 0) for (auto& p : row) p.rank += perturb(rng);
            std::sort(row.begin(), row.end(), placement_less);
        }

        std::size_t choice = 0;
        if (op == 2) {
            choice = std::uniform_int_distribution<std::size_t>(0, rows.size() - 1)(rng);
        } else {
            auto regret = [](const std::vector<Placement>& row) {
                return row.size() == 1 ? std::numeric_limits<double>::infinity() : row[1].rank - row[0].rank;
            };
            for (std::size_t i = 1; i < rows.size(); ++i) {
                const bool better = op == 0 ? placement_less(rows[i].front(), rows[choice].front())
                                            : regret(rows[i]) > regret(rows[choice]);
                if (better) choice = i;
            }
        }
        insert(plan, rows[choice].front());
        pending.erase(pending.begin() + choice);
    }
    return true;
}

std::vector<int> choose_stops(const Instance& in, const Plan& plan, int limit) {
    int anchor = -1;
    double max_wait = -1;
    for (int i = 0; i < static_cast<int>(plan.size()); ++i) {
        const auto [duration, sortie] = longest_flight(in, plan[i]);
        if (sortie && duration > max_wait) {
            anchor = i;
            max_wait = duration;
        }
    }
    if (anchor < 0) return {};
    auto count = [&](int i) {
        int total = 0;
        for (const auto& sortie : plan[i].sorties) total += static_cast<int>(sortie.size());
        return total;
    };
    if (count(anchor) > limit) return {};
    int adjacent = anchor - 1;
    const int next = anchor + 1;
    if (next < static_cast<int>(plan.size()) &&
        (adjacent < 0 || in.road[plan[anchor].node][plan[next].node] < in.road[plan[anchor].node][plan[adjacent].node])) {
        adjacent = next;
    }
    if (adjacent >= 0 && count(anchor) + count(adjacent) <= limit) return {anchor, adjacent};
    return {anchor};
}

}  // namespace

Plan make_initial_plan(const Instance& instance, Clock::time_point deadline) {
    return initial_plan(instance, deadline);
}

SearchResult solve_alns(const Instance& instance, const Settings& settings, bool allow_cross_stop,
                        const IncumbentSink& incumbent, const OperatorSink& operators,
                        const NeighborhoodSink& neighborhood) {
    const auto start = Clock::now();
    const auto deadline = start + std::chrono::milliseconds(settings.wall_ms);
    Random rng(settings.seed);
    Plan current = make_initial_plan(instance, deadline);
    Evaluation current_eval = evaluate(instance, current);
    SearchResult result;
    result.best = current;
    result.evaluation = current_eval;
    result.initial_cost = current_eval.total;
    auto report_incumbent = [&] {
        const auto elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        incumbent(result.best, result.evaluation, elapsed);
    };
    report_incumbent();

    Operators<6> destroy_operators;
    Operators<3> repair_operators;
    Operators<2> noise_operators;
    double temperature = settings.initial_temperature;
    int without_improvement = 0;
    auto update_best = [&] {
        if (current_eval.total < result.evaluation.total) {
            result.best = current;
            result.evaluation = current_eval;
            without_improvement = 0;
            report_incumbent();
        }
    };

    while (result.iterations < settings.iterations && Clock::now() < deadline && temperature > settings.min_temperature) {
        const double fraction = 1.0 / (1.0 + std::exp(-settings.sigmoid_alpha * (without_improvement - settings.sigmoid_beta)));
        const int q = static_cast<int>(std::lround(settings.q_min + (settings.q_max - settings.q_min) * fraction));
        const int d = destroy_operators.select(rng);
        const int r = repair_operators.select(rng);
        const int n = noise_operators.select(rng);
        Plan trial = current;
        const auto destroy_start = Clock::now();
        const auto pending = destroy(instance, trial, d, q, rng, deadline);
        const auto repair_start = Clock::now();
        const bool completed = repair(instance, trial, pending, r, n ? settings.noise : 0.0, rng, deadline);
        const auto repair_end = Clock::now();
        const auto destroy_us = std::chrono::duration<double, std::micro>(repair_start - destroy_start).count();
        const auto repair_us = std::chrono::duration<double, std::micro>(repair_end - repair_start).count();
        OperatorEvent event{result.iterations + 1, d, r, n, q, static_cast<int>(pending.size()),
                            completed && repair_end < deadline, false, false, false, destroy_us, repair_us};
        if (!event.completed) {
            operators(event);
            break;
        }
        const auto trial_eval = evaluate(instance, trial);
        const double difference = trial_eval.total - current_eval.total;
        const bool accepted = trial_eval.feasible && (difference <= 0 ||
            std::uniform_real_distribution<double>(0.0, 1.0)(rng) < std::exp(-difference / temperature));
        const std::size_t outcome = accepted ? (trial_eval.total < result.evaluation.total ? 0 : (difference < 0 ? 1 : 2)) : 3;
        event.accepted = accepted;
        event.improved_current = accepted && difference < 0;
        event.improved_best = outcome == 0;
        operators(event);
        destroy_operators.record(d, settings.rewards[outcome]);
        repair_operators.record(r, settings.rewards[outcome]);
        noise_operators.record(n, settings.rewards[outcome]);

        ++result.iterations;
        ++without_improvement;
        if (accepted) {
            current = std::move(trial);
            current_eval = trial_eval;
            update_best();
        }
        if (settings.exact_every > 0 && result.iterations % settings.exact_every == 0 && Clock::now() < deadline) {
            const auto selected = choose_stops(instance, current, settings.exact_pool);
            if (!selected.empty()) {
                neighborhood(result.iterations, current, selected);
                ++result.exact_calls;
                const auto local_deadline = std::min(deadline, Clock::now() + std::chrono::milliseconds(settings.exact_ms));
                if (improve_exact(instance, current, selected, local_deadline, allow_cross_stop)) {
                    ++result.exact_complete;
                }
                const auto improved = evaluate(instance, current);
                if (improved.total < current_eval.total) ++result.exact_improvements;
                current_eval = improved;
                update_best();
            }
        }
        if (result.iterations % settings.update_every == 0) {
            destroy_operators.update(settings.decay);
            repair_operators.update(settings.decay);
            noise_operators.update(settings.decay);
        }
        temperature *= settings.cooling;
    }
    result.elapsed_s = std::chrono::duration<double>(Clock::now() - start).count();
    return result;
}

}  // namespace routing
