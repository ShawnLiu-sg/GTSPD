#include "validation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

namespace validation {
namespace {

Flight flight_forward(const Instance& in, int anchor, const Sortie& route) {
    double payload = 0;
    for (int node : route) payload += in.customers[node].mass;
    if (payload > in.drone_capacity + 1e-10) return {};
    Flight f;
    int previous = anchor;
    auto leg = [&](int next) {
        const double length = in.air[previous][next];
        const double time = length / in.drone_speed;
        f.distance += length;
        f.energy += (in.drone_empty_w + in.drone_payload_w_per_kg * payload) * time / 3600000.0;
        previous = next;
    };
    for (int node : route) {
        leg(node);
        payload -= in.customers[node].mass;
    }
    payload = 0;
    leg(anchor);
    f.duration = f.distance / in.drone_speed;
    f.feasible = f.energy <= in.drone_budget + 1e-10;
    return f;
}

bool eligible(const Instance& in, int node) {
    return in.customers[node].drone_eligible && in.customers[node].mass <= in.drone_capacity;
}

}  // namespace

Flight evaluate_flight_forward(const Instance& in, int anchor, const Sortie& route) {
    if (route.empty() || !in.customers[anchor].launch_allowed) return {};
    for (int node : route) if (node == anchor || !eligible(in,node)) return {};
    return flight_forward(in,anchor,route);
}

Evaluation evaluate_forward(const Instance& in, const Plan& plan) {
    if (plan.empty()) return {};
    const int n = static_cast<int>(in.customers.size());
    std::vector<bool> visited(n);
    visited[0] = true;
    double payload = 0;
    for (int i = 1; i < n; ++i) payload += in.customers[i].mass;
    if (payload > in.truck_capacity + 1e-10) return {};

    Evaluation out;
    int previous = 0;
    auto road_leg = [&](int next) {
        const double length = in.road[previous][next];
        out.truck_distance += length;
        out.fuel_l += (in.truck_empty_l_per_s + in.truck_payload_l_per_s_kg * payload)
                      * length / in.truck_speed;
        previous = next;
    };
    for (const Stop& stop : plan) {
        if (stop.node < 1 || stop.node >= n || visited[stop.node] ||
            static_cast<int>(stop.sorties.size()) > in.drones ||
            (!stop.sorties.empty() && !in.customers[stop.node].launch_allowed)) return {};
        road_leg(stop.node);
        visited[stop.node] = true;
        payload -= in.customers[stop.node].mass;
        double dwell = 0;
        for (const Sortie& sortie : stop.sorties) {
            if (sortie.empty()) return {};
            for (int node : sortie) {
                if (node < 1 || node >= n || visited[node] || !eligible(in, node)) return {};
                visited[node] = true;
                payload -= in.customers[node].mass;
            }
            const Flight flight = flight_forward(in, stop.node, sortie);
            if (!flight.feasible) return {};
            out.drone_distance += flight.distance;
            out.energy_kwh += flight.energy;
            dwell = std::max(dwell, flight.duration);
        }
        out.wait_s += dwell;
    }
    if (std::find(visited.begin(), visited.end(), false) != visited.end()) return {};
    payload = 0;
    road_leg(0);
    out.operating_cost = in.truck_cost_per_m * out.truck_distance + in.drone_cost_per_m * out.drone_distance;
    out.carbon_kg = in.fuel_co2_per_l * out.fuel_l + in.electricity_co2_per_kwh * out.energy_kwh;
    out.carbon_cost = in.carbon_cost_per_kg * out.carbon_kg;
    out.waiting_cost = in.wait_cost_per_s * out.wait_s;
    out.total = out.operating_cost + out.carbon_cost + out.waiting_cost;
    out.feasible = true;
    return out;
}

OracleResult exhaustive(const Instance& in) {
    using Mask = unsigned;
    struct Pattern { Mask mask; Sortie route; };
    const int n = static_cast<int>(in.customers.size()) - 1;
    const Mask all = (Mask{1} << n) - 1;
    Mask mandatory = 0;
    const auto bit = [](int node) { return Mask{1} << (node - 1); };
    for (int i = 1; i <= n; ++i) if (!eligible(in, i)) mandatory |= bit(i);
    std::vector<std::vector<Pattern>> patterns(n + 1);
    if (in.drones != 0) {
        for (int v = 1; v <= n; ++v) {
            if (!in.customers[v].launch_allowed) continue;
            const Mask allowed = all ^ (mandatory | bit(v));
            for (Mask subset = allowed; subset != 0; subset = (subset - 1) & allowed) {
                Sortie route;
                for (int i = 1; i <= n; ++i) if (subset & bit(i)) route.push_back(i);
                do {
                    if (flight_forward(in, v, route).feasible) patterns[v].push_back({subset, route});
                } while (std::next_permutation(route.begin(), route.end()));
            }
        }
    }
    OracleResult out;
    out.checked.total = infinity;
    for (Mask truck = 1; truck <= all; ++truck) {
        if ((truck & mandatory) != mandatory) continue;
        std::vector<int> order;
        for (int i = 1; i <= n; ++i) if (truck & bit(i)) order.push_back(i);
        do {
            ++out.truck_tours;
            Plan trial;
            for (int node : order) trial.push_back({node,{}});
            const auto allocate = [&](auto&& recurse, Mask remaining) -> void {
                if (remaining == 0) {
                    const auto verified = evaluate_forward(in, trial);
                    if (!verified.feasible) return;
                    ++out.complete_assignments;
                    if (verified.total < out.checked.total) {
                        out.checked = verified;
                        out.plan = trial;
                    }
                    return;
                }
                const Mask pivot = remaining & (Mask{0} - remaining);
                for (Stop& stop : trial) {
                    if (static_cast<int>(stop.sorties.size()) == in.drones) continue;
                    for (const Pattern& p : patterns[stop.node]) {
                        if (!(p.mask & pivot) || (p.mask & remaining) != p.mask) continue;
                        stop.sorties.push_back(p.route);
                        recurse(recurse, remaining ^ p.mask);
                        stop.sorties.pop_back();
                    }
                }
            };
            allocate(allocate, all ^ truck);
        } while (std::next_permutation(order.begin(), order.end()));
    }
    return out;
}

}  // namespace validation
