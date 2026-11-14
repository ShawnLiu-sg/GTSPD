#include "routing.hpp"

#include <algorithm>

namespace routing {

double flight_leg_energy(const Instance& instance, double distance, double mass) {
    constexpr double joules_per_kwh = 3600000.0;
    const double power = instance.drone_empty_w + instance.drone_payload_w_per_kg * mass;
    return power * distance / (instance.drone_speed * joules_per_kwh);
}

Flight evaluate_sortie(const Instance& instance, int anchor, const Sortie& sortie) {
    const int node_count = static_cast<int>(instance.customers.size());
    if (anchor <= 0 || anchor >= node_count || !instance.customers[anchor].launch_allowed ||
        sortie.empty()) {
        return {};
    }

    Flight flight;
    double load = 0.0;
    int next = anchor;
    // Backward traversal accumulates the parcels still aboard after each delivery.
    for (auto visit = sortie.rbegin(); visit != sortie.rend(); ++visit) {
        const int node = *visit;
        if (node <= 0 || node >= node_count || node == anchor ||
            !instance.customers[node].drone_eligible ||
            std::find(sortie.rbegin(), visit, node) != visit) {
            return {};
        }
        const double distance = instance.air[node][next];
        flight.distance += distance;
        flight.energy += flight_leg_energy(instance, distance, load);
        load += instance.customers[node].mass;
        next = node;
    }
    const double departure_distance = instance.air[anchor][next];
    flight.distance += departure_distance;
    flight.energy += flight_leg_energy(instance, departure_distance, load);
    flight.duration = flight.distance / instance.drone_speed;
    flight.feasible = load <= instance.drone_capacity && flight.energy <= instance.drone_budget;
    return flight;
}

Evaluation evaluate(const Instance& instance, const Plan& plan, bool complete) {
    Evaluation result;
    std::vector<bool> served(instance.customers.size(), false);
    const int node_count = static_cast<int>(served.size());
    int served_count = 0;
    double load = 0.0;
    int next = 0;

    for (auto stop = plan.rbegin(); stop != plan.rend(); ++stop) {
        const int node = stop->node;
        if (node <= 0 || node >= node_count || served[node] ||
            stop->sorties.size() > static_cast<std::size_t>(instance.drones)) {
            return {};
        }

        const double distance = instance.road[node][next];
        result.truck_distance += distance;
        result.fuel_l += (instance.truck_empty_l_per_s + instance.truck_payload_l_per_s_kg * load)
                         * distance / instance.truck_speed;
        load += instance.customers[node].mass;
        served[node] = true;
        ++served_count;

        double dwell = 0.0;
        for (const Sortie& sortie : stop->sorties) {
            const Flight flight = evaluate_sortie(instance, node, sortie);
            if (!flight.feasible) {
                return {};
            }
            for (int customer : sortie) {
                if (served[customer]) {
                    return {};
                }
                load += instance.customers[customer].mass;
                served[customer] = true;
                ++served_count;
            }
            result.drone_distance += flight.distance;
            result.energy_kwh += flight.energy;
            dwell = std::max(dwell, flight.duration);
        }
        result.wait_s += dwell;
        next = node;
    }
    if (complete && served_count != node_count - 1) {
        return {};
    }
    if (!plan.empty()) {
        const double distance = instance.road[0][next];
        result.truck_distance += distance;
        result.fuel_l += (instance.truck_empty_l_per_s + instance.truck_payload_l_per_s_kg * load)
                         * distance / instance.truck_speed;
    }

    result.operating_cost = instance.truck_cost_per_m * result.truck_distance
                            + instance.drone_cost_per_m * result.drone_distance;
    result.carbon_kg = instance.fuel_co2_per_l * result.fuel_l
                       + instance.electricity_co2_per_kwh * result.energy_kwh;
    result.carbon_cost = instance.carbon_cost_per_kg * result.carbon_kg;
    result.waiting_cost = instance.wait_cost_per_s * result.wait_s;
    result.total = result.operating_cost + result.carbon_cost + result.waiting_cost;
    result.feasible = true;
    return result;
}

}  // namespace routing
