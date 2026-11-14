#pragma once

#include <chrono>
#include <vector>

namespace routing {

using Clock = std::chrono::steady_clock;
using Sortie = std::vector<int>;

struct Customer {
    double mass{};
    bool drone_eligible{};
    bool launch_allowed{};
};

struct Instance {
    std::vector<Customer> customers;
    std::vector<std::vector<double>> road;
    std::vector<std::vector<double>> air;
    int drones{};
    double truck_capacity{}, drone_capacity{}, drone_budget{};
    double truck_speed{}, drone_speed{};
    double truck_cost_per_m{}, drone_cost_per_m{}, carbon_cost_per_kg{}, wait_cost_per_s{};
    double fuel_co2_per_l{}, electricity_co2_per_kwh{};
    double truck_empty_l_per_s{}, truck_payload_l_per_s_kg{};
    double drone_empty_w{}, drone_payload_w_per_kg{};
};

struct Stop {
    int node;
    std::vector<Sortie> sorties;
};

using Plan = std::vector<Stop>;

struct Flight {
    bool feasible{};
    double distance{}, energy{}, duration{};
};

struct Evaluation {
    bool feasible{};
    double truck_distance{}, drone_distance{}, fuel_l{}, energy_kwh{}, wait_s{};
    double operating_cost{}, carbon_kg{}, carbon_cost{}, waiting_cost{}, total{};
};

// Distances in metres, powers in watts, energy in kWh.
double flight_leg_energy(const Instance& instance, double distance, double mass);
Flight evaluate_sortie(const Instance& instance, int anchor, const Sortie& sortie);
Evaluation evaluate(const Instance& instance, const Plan& plan, bool complete = true);

// Indices are zero-based positions in the truck plan. True certifies local completion.
// The selected stops remain on the truck tour and the entire set of their drone customers is pooled.
bool improve_exact(const Instance& instance, Plan& plan, const std::vector<int>& stop_indices,
                   Clock::time_point deadline, bool allow_cross_stop = true);

}  // namespace routing
