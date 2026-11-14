#include "benchmark.hpp"

#define NOMINMAX
#include <windows.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void write_metrics(const routing::Evaluation& value) {
    if (!value.feasible) {
        std::cout << "null";
        return;
    }
    std::cout << "{\"objective\":" << value.total
              << ",\"operating_cost\":" << value.operating_cost
              << ",\"carbon_cost\":" << value.carbon_cost
              << ",\"waiting_cost\":" << value.waiting_cost
              << ",\"emissions_kg_co2e\":" << value.carbon_kg
              << ",\"truck_m\":" << value.truck_distance
              << ",\"air_m\":" << value.drone_distance
              << ",\"fuel_l\":" << value.fuel_l
              << ",\"air_kwh\":" << value.energy_kwh
              << ",\"wait_s\":" << value.wait_s << '}';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 13) {
            throw std::runtime_error("Usage: operate input policy D Q E speed carbon wait grid seed ms core");
        }
        const auto core = std::stoi(argv[12]);
        if (!SetProcessAffinityMask(GetCurrentProcess(), DWORD_PTR{1} << core)) {
            throw std::runtime_error("Unable to set the requested processor affinity");
        }

        std::ifstream input(argv[1]);
        if (!input) throw std::runtime_error("Cannot open case");
        auto physical = routing::read_instance(input);
        auto settings = routing::read_settings(input, static_cast<int>(physical.customers.size()) - 1);
        const std::string policy = argv[2];
        physical.drones = std::stoi(argv[3]);
        physical.drone_capacity = std::stod(argv[4]);
        physical.drone_budget = std::stod(argv[5]);
        physical.drone_speed = std::stod(argv[6]);
        if (physical.drone_speed == 13.41) {
            physical.drone_empty_w = 1238.9252876087053;
            physical.drone_payload_w_per_kg = 185.4903714817606;
        }
        physical.carbon_cost_per_kg = std::stod(argv[7]);
        physical.wait_cost_per_s = std::stod(argv[8]);
        physical.electricity_co2_per_kwh = std::stod(argv[9]);
        settings.seed = std::stoull(argv[10]);
        settings.wall_ms = std::stoi(argv[11]);

        auto planning = physical;
        if (policy == "single_drop") {
            planning.single_drop = true;
        } else if (policy == "no_wait") {
            planning.wait_cost_per_s = 0.0;
        } else if (policy == "constant_energy") {
            // Freeze the surrogate at the midpoint of the base payload range.
            planning.drone_empty_w += planning.drone_payload_w_per_kg * 2.25;
            planning.drone_payload_w_per_kg = 0.0;
        } else if (policy == "emissions_only") {
            planning.truck_cost_per_m = 0.0;
            planning.drone_cost_per_m = 0.0;
            planning.wait_cost_per_s = 0.0;
        }

        routing::Plan best;
        routing::Evaluation best_value;
        const auto solution = routing::solve_alns(planning, settings, true,
            [&](const routing::Plan& plan, const routing::Evaluation& value, double ms) {
                if (ms <= settings.wall_ms && value.feasible) {
                    best = plan;
                    best_value = value;
                }
            });
        int drones_used = 0;
        int drone_orders = 0;
        int overrun_count = 0;
        double max_overrun = 0;
        for (const auto& stop : best) {
            drones_used = std::max(drones_used, static_cast<int>(stop.sorties.size()));
            for (const auto& flight : stop.sorties) {
                drone_orders += static_cast<int>(flight.size());
                const auto check = routing::evaluate_sortie(physical, stop.node, flight);
                const double overrun = check.energy - physical.drone_budget;
                if (overrun > 0) {
                    ++overrun_count;
                    max_overrun = std::max(max_overrun, overrun);
                }
            }
        }

        std::cout << std::setprecision(15)
                  << "{\"cpu\":" << core << ",\"iterations\":" << solution.iterations
                  << ",\"elapsed_ms\":" << solution.elapsed_s * 1000
                  << ",\"exact_calls\":" << solution.exact_calls
                  << ",\"exact_complete\":" << solution.exact_complete
                  << ",\"drone_orders\":" << drone_orders
                  << ",\"peak_drones_used\":" << drones_used
                  << ",\"battery_overrun_count\":" << overrun_count
                  << ",\"max_battery_overrun_kwh\":" << max_overrun
                  << ",\"planning\":";
        write_metrics(best_value);
        std::cout << ",\"physical\":";
        write_metrics(routing::evaluate(physical, best));
        std::cout << ",\"plan\":[";
        bool first_stop = true;
        for (const auto& stop : best) {
            if (!first_stop) std::cout << ',';
            first_stop = false;
            std::cout << '[' << stop.node;
            for (const auto& sortie : stop.sorties) {
                std::cout << ",[";
                bool first_customer = true;
                for (int customer : sortie) {
                    if (!first_customer) std::cout << ',';
                    first_customer = false;
                    std::cout << customer;
                }
                std::cout << ']';
            }
            std::cout << ']';
        }
        std::cout << "]}\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
