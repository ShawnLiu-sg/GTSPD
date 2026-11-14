#include "benchmark.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace routing;
using Mask = unsigned int;
constexpr double infinity = std::numeric_limits<double>::infinity();

struct Candidate {
    Mask served;
    double cost;
    double flight_seconds;
};

struct Snapshot {
    int iteration = 0;
    Plan plan;
    std::vector<int> stops;
};

struct Oracle {
    const Instance& in;
    const Snapshot& snapshot;
    std::vector<int> pool;
    std::vector<std::vector<std::vector<Candidate>>> options;
    Mask first_current;
    double outside_cost;

    Oracle(const Instance& instance, const Snapshot& captured)
        : in(instance), snapshot(captured), first_current(0), outside_cost(0) {
        double truck_distance = 0;
        int prior = 0;
        Plan outside = snapshot.plan;
        for (const int index : snapshot.stops) {
            for (const Sortie& sortie : snapshot.plan[index].sorties) {
                for (const int order : sortie) {
                    if (index == snapshot.stops.front()) first_current |= Mask{1} << pool.size();
                    pool.push_back(order);
                }
            }
            outside[index].sorties.clear();
        }
        const double truck_price = in.truck_cost_per_m;
        const double fuel_price = in.carbon_cost_per_kg * in.fuel_co2_per_l;
        const double power_price = in.carbon_cost_per_kg * in.electricity_co2_per_kwh;
        std::vector<double> prefix(snapshot.plan.size(), 0.0);
        for (std::size_t index = 0; index < snapshot.plan.size(); ++index) {
            const int node = snapshot.plan[index].node;
            truck_distance += in.road[prior][node];
            prefix[index] = truck_distance;
            prior = node;
        }
        truck_distance += in.road[prior][0];
        const double fuel_per_mass_m = in.truck_payload_l_per_s_kg / in.truck_speed;
        outside_cost = truck_distance * (truck_price + fuel_price * in.truck_empty_l_per_s / in.truck_speed);
        for (std::size_t index = 0; index < outside.size(); ++index) {
            const Stop& stop = outside[index];
            double payload = in.customers[stop.node].mass;
            double wait = 0.0;
            for (const Sortie& trip : stop.sorties) {
                const auto [distance, energy, mass] = flight(stop.node, trip);
                payload += mass;
                wait = std::max(wait, distance / in.drone_speed);
                outside_cost += in.drone_cost_per_m * distance + power_price * energy;
            }
            outside_cost += fuel_price * fuel_per_mass_m * prefix[index] * payload + in.wait_cost_per_s * wait;
        }
        options.resize(snapshot.stops.size());
        for (std::size_t s = 0; s < snapshot.stops.size(); ++s) {
            options[s].resize(pool.size());
            const int index = snapshot.stops[s];
            const int anchor = snapshot.plan[index].node;
            for (Mask mask = 1; mask < (Mask{1} << pool.size()); ++mask) {
                std::vector<int> path;
                std::size_t pivot = pool.size();
                for (std::size_t i = 0; i < pool.size(); ++i) {
                    if (!(mask & (Mask{1} << i))) continue;
                    pivot = std::min(pivot, i);
                    path.push_back(pool[i]);
                }
                std::sort(path.begin(), path.end());
                const double payload = std::accumulate(path.begin(), path.end(), 0.0, [&](double mass, int node) {
                    return mass + in.customers[node].mass;
                });
                if (payload > in.drone_capacity) continue;
                do {
                    const auto [distance, energy, mass] = flight(anchor, path);
                    if (energy > in.drone_budget) continue;
                    const double cost = in.drone_cost_per_m * distance + power_price * energy
                                        + fuel_price * fuel_per_mass_m * prefix[index] * mass;
                    options[s][pivot].push_back({mask, cost, distance / in.drone_speed});
                } while (std::next_permutation(path.begin(), path.end()));
            }
        }
    }

    struct Trip {
        double distance, energy, mass;
    };

    Trip flight(int anchor, const Sortie& customers) const {
        const double mass = std::accumulate(customers.begin(), customers.end(), 0.0, [&](double load, int node) {
            return load + in.customers[node].mass;
        });
        double remaining = mass;
        double distance = 0.0;
        double energy = 0.0;
        int previous = anchor;
        for (int node : customers) {
            const double leg = in.air[previous][node];
            distance += leg;
            energy += (in.drone_empty_w + remaining * in.drone_payload_w_per_kg) * leg;
            remaining -= in.customers[node].mass;
            previous = node;
        }
        distance += in.air[previous][anchor];
        energy += in.drone_empty_w * in.air[previous][anchor];
        return {distance, energy / in.drone_speed / 3600000.0, mass};
    }

    std::vector<double> solve_stop(int stop) const {
        const Mask states = Mask{1} << pool.size();
        std::vector<double> best(states, infinity);
        best[0] = 0.0;
        for (Mask assigned = 1; assigned < best.size(); ++assigned) {
            const auto enumerate = [&](auto&& visit, Mask remaining, int slots, double cost, double longest) -> void {
                const double bound = cost + in.wait_cost_per_s * longest;
                if (bound >= best[assigned]) return;
                if (!remaining) {
                    best[assigned] = bound;
                    return;
                }
                if (!slots) return;
                std::size_t pivot = 0;
                while (!(remaining & (Mask{1} << pivot))) ++pivot;
                for (const Candidate& choice : options[stop][pivot]) {
                    if ((choice.served & remaining) != choice.served) continue;
                    visit(visit, remaining ^ choice.served, slots - 1, cost + choice.cost,
                          std::max(longest, choice.flight_seconds));
                }
            };
            enumerate(enumerate, assigned, in.drones, 0.0, 0.0);
        }
        return best;
    }

    std::pair<double, double> solve() const {
        const auto first = solve_stop(0);
        const auto second = solve_stop(1);
        const Mask all = static_cast<Mask>(first.size() - 1);
        double unrestricted = infinity;
        for (Mask assigned = 0; assigned <= all; ++assigned) {
            unrestricted = std::min(unrestricted, first[assigned] + second[all ^ assigned]);
        }
        return {outside_cost + first[first_current] + second[all ^ first_current], outside_cost + unrestricted};
    }
};

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

int neighborhood_size(const Plan& plan, const std::vector<int>& stops) {
    int count = 0;
    for (int stop : stops) {
        for (const Sortie& trip : plan[stop].sorties) count += static_cast<int>(trip.size());
    }
    return count;
}

void write_plan(const Plan& plan) {
    std::cout << '\t';
    for (const Stop& stop : plan) std::cout << stop.node << ' ';
    std::cout << '\t';
    bool separator = false;
    for (const Stop& stop : plan) {
        for (const Sortie& trip : stop.sorties) {
            if (separator) std::cout << '|';
            std::cout << stop.node << ':';
            for (int node : trip) std::cout << node << ' ';
            separator = true;
        }
    }
}

void replay(const Instance& instance, const Snapshot& snapshot) {
    const Evaluation original = evaluate(instance, snapshot.plan);
    const int pool = neighborhood_size(snapshot.plan, snapshot.stops);
    const int reps = 7;
    std::vector<double> oracle_us, fixed_us, cross_us;
    Evaluation fixed, cross;
    std::pair<double, double> reference;
    Plan fixed_plan, cross_plan;
    for (int rep = 0; rep < reps; ++rep) {
        auto exact = [&](bool allowed, std::vector<double>& times, Plan& solution, Evaluation& evaluation) {
            solution = snapshot.plan;
            const auto start = Clock::now();
            const bool complete = improve_exact(instance, solution, snapshot.stops, start + std::chrono::seconds(10), allowed);
            times.push_back(std::chrono::duration<double, std::micro>(Clock::now() - start).count());
            evaluation = evaluate(instance, solution);
            if (!complete || !evaluation.feasible) throw std::runtime_error("Incomplete or infeasible replay");
        };
        if (rep % 2 == 0) {
            exact(false, fixed_us, fixed_plan, fixed);
            exact(true, cross_us, cross_plan, cross);
        } else {
            exact(true, cross_us, cross_plan, cross);
            exact(false, fixed_us, fixed_plan, fixed);
        }
        const auto start = Clock::now();
        reference = Oracle(instance, snapshot).solve();
        oracle_us.push_back(std::chrono::duration<double, std::micro>(Clock::now() - start).count());
        const auto differs = [](double left, double right) {
            return !std::isfinite(left) || !std::isfinite(right) || std::abs(left - right) > 1e-8;
        };
        if (!original.feasible || differs(fixed.total, reference.first) || differs(cross.total, reference.second) ||
            fixed.total > original.total + 1e-8 || cross.total > fixed.total + 1e-8) {
            throw std::runtime_error("Fixed or cross-stop replay disagrees with exhaustive enumeration");
        }
    }
    std::cout << "snapshot\t" << snapshot.iteration << '\t'
              << snapshot.plan[snapshot.stops[0]].node << '\t' << snapshot.plan[snapshot.stops[1]].node << '\t'
              << pool << '\t' << original.total << '\t' << fixed.total << '\t' << cross.total << '\t'
              << reference.first << '\t' << reference.second << '\t'
              << original.wait_s << '\t' << fixed.wait_s << '\t' << cross.wait_s << '\t'
              << original.carbon_kg << '\t' << fixed.carbon_kg << '\t' << cross.carbon_kg << '\t'
              << median(fixed_us) << '\t' << median(cross_us) << '\t' << median(oracle_us);
    write_plan(snapshot.plan);
    write_plan(fixed_plan);
    write_plan(cross_plan);
    std::cout << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: replay <instance> <seed> <max_ms>\n";
        return 2;
    }
    try {
        std::ifstream source(argv[1]);
        const Instance instance = read_instance(source);
        Settings settings = read_settings(source, static_cast<int>(instance.customers.size()) - 1);
        settings.seed = std::stoull(argv[2]);
        settings.wall_ms = std::stoi(argv[3]);
        Snapshot snapshot;
        std::vector<OperatorEvent> events;
        const auto observe_operators = [&](const OperatorEvent& event) { events.push_back(event); };
        const auto observe_neighborhood = [&](int iteration, const Plan& plan, const std::vector<int>& stops) {
            if (snapshot.iteration || iteration < 20 || stops.size() != 2 ||
                !instance.customers[plan[stops[1]].node].launch_allowed) return;
            const int pool = neighborhood_size(plan, stops);
            if (pool < 2 || pool > 8) return;
            snapshot = {iteration, plan, stops};
        };
        const auto search = solve_alns(instance, settings, true,
                                       [](const Plan&, const Evaluation&, double) {},
                                       observe_operators, observe_neighborhood);
        std::cout << std::setprecision(15);
        std::cout << "run\t" << search.iterations << '\t' << search.elapsed_s << '\t' << search.exact_calls
                  << '\t' << search.exact_complete << '\t' << search.exact_improvements << '\n';
        for (const auto& e : events) {
            std::cout << "operator\t" << e.iteration << '\t' << e.destroy << '\t' << e.repair << '\t' << e.noise << '\t'
                      << e.requested << '\t' << e.removed << '\t' << e.completed << '\t' << e.accepted << '\t'
                      << e.improved_current << '\t' << e.improved_best << '\t' << e.destroy_us << '\t' << e.repair_us << '\n';
        }
        if (snapshot.iteration) replay(instance, snapshot);
        else std::cout << "no_snapshot\n";
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << '\n';
        return 1;
    }
}
