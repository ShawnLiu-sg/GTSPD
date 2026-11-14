#include "search.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
#include <string>

namespace {

using namespace routing;

template<class T>
T read(std::istream& input, const char* name) {
    T value{};
    if (!(input >> value)) throw std::runtime_error(std::string("Expected ") + name);
    return value;
}

void expect(std::istream& input, const char* expected) {
    if (read<std::string>(input, expected) != expected) {
        throw std::runtime_error(std::string("Expected section ") + expected);
    }
}

double number(std::istream& input, const char* name, bool positive = false) {
    const auto value = read<double>(input, name);
    if (!std::isfinite(value) || value < 0 || (positive && value == 0)) {
        throw std::runtime_error(std::string("Invalid value for ") + name);
    }
    return value;
}

int integer(std::istream& input, const char* name, int minimum) {
    const auto value = read<int>(input, name);
    if (value < minimum) throw std::runtime_error(std::string("Invalid value for ") + name);
    return value;
}

bool flag(std::istream& input, const char* name) {
    const auto value = integer(input, name, 0);
    if (value > 1) throw std::runtime_error(std::string("Expected 0 or 1 for ") + name);
    return value == 1;
}

std::vector<std::vector<double>> read_matrix(std::istream& input, int size) {
    std::vector<std::vector<double>> matrix(size, std::vector<double>(size));
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            matrix[i][j] = number(input, "distance in metres");
            if (i == j && matrix[i][j] != 0) throw std::runtime_error("Distance matrix diagonal must be zero");
        }
    }
    return matrix;
}

Instance read_instance(std::istream& input) {
    expect(input, "STDRP");
    expect(input, "1");
    expect(input, "CUSTOMERS");
    const int n = integer(input, "customer count", 1);
    Instance in;
    expect(input, "TRUCK");
    in.truck_capacity = number(input, "truck payload capacity (kg)", true);
    in.truck_speed = number(input, "truck speed (m/s)", true);
    in.truck_empty_l_per_s = number(input, "empty truck fuel rate (L/s)");
    in.truck_payload_l_per_s_kg = number(input, "truck fuel rate per kg (L/s/kg)");
    expect(input, "DRONE");
    in.drones = integer(input, "drones per stop", 0);
    in.drone_capacity = number(input, "drone payload capacity (kg)", true);
    in.drone_budget = number(input, "usable drone battery (kWh)", true);
    in.drone_speed = number(input, "drone speed (m/s)", true);
    in.drone_empty_w = number(input, "empty drone power (W)");
    in.drone_payload_w_per_kg = number(input, "drone power per kg (W/kg)");
    expect(input, "COSTS");
    in.truck_cost_per_m = number(input, "truck operating cost per metre");
    in.drone_cost_per_m = number(input, "drone operating cost per metre");
    in.carbon_cost_per_kg = number(input, "carbon charge per kg CO2e");
    in.wait_cost_per_s = number(input, "waiting charge per second");
    in.fuel_co2_per_l = number(input, "truck fuel factor (kg CO2e/L)");
    in.electricity_co2_per_kwh = number(input, "battery energy factor (kg CO2e/kWh)");

    expect(input, "ORDERS");
    in.customers.resize(static_cast<std::size_t>(n) + 1);
    double mass = 0;
    for (int i = 1; i <= n; ++i) {
        if (read<int>(input, "customer ID") != i) throw std::runtime_error("Customer IDs must be 1 through n in order");
        auto& c = in.customers[i];
        c.mass = number(input, "parcel mass (kg)", true);
        c.drone_eligible = flag(input, "drone eligibility");
        c.launch_allowed = flag(input, "launch permission");
        mass += c.mass;
    }
    if (mass > in.truck_capacity) throw std::runtime_error("The truck must carry all parcels at departure");

    const auto format = read<std::string>(input, "COORDINATES or ROAD");
    if (format == "COORDINATES") {
        std::vector<std::pair<double, double>> points(in.customers.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (read<std::size_t>(input, "coordinate ID") != i) throw std::runtime_error("Coordinate IDs must be 0 through n in order");
            points[i] = {read<double>(input, "x in metres"), read<double>(input, "y in metres")};
            if (!std::isfinite(points[i].first) || !std::isfinite(points[i].second)) {
                throw std::runtime_error("Coordinates must be finite");
            }
        }
        in.road.assign(points.size(), std::vector<double>(points.size()));
        in.air = in.road;
        for (std::size_t i = 0; i < points.size(); ++i) {
            for (std::size_t j = 0; j < points.size(); ++j) {
                const double x = points[i].first - points[j].first;
                const double y = points[i].second - points[j].second;
                in.road[i][j] = std::abs(x) + std::abs(y);
                in.air[i][j] = std::hypot(x, y);
            }
        }
    } else if (format == "ROAD") {
        in.road = read_matrix(input, n + 1);
        expect(input, "AIR");
        in.air = read_matrix(input, n + 1);
    } else {
        throw std::runtime_error("Expected COORDINATES or ROAD");
    }
    return in;
}

Settings read_settings(std::istream& input, int n) {
    Settings settings;
    expect(input, "SEARCH");
    settings.seed = read<std::uint64_t>(input, "random seed");
    settings.iterations = integer(input, "iteration limit", 0);
    settings.wall_ms = integer(input, "wall-clock budget (ms)", 1);
    expect(input, "DESTROY");
    settings.q_min = integer(input, "minimum destruction size", 1);
    settings.q_max = integer(input, "maximum destruction size", settings.q_min);
    settings.sigmoid_alpha = number(input, "sigmoid slope", true);
    settings.sigmoid_beta = number(input, "sigmoid midpoint");
    if (settings.q_max > n) throw std::runtime_error("Destruction size exceeds number of customers");
    expect(input, "ANNEAL");
    settings.initial_temperature = number(input, "initial temperature", true);
    settings.cooling = number(input, "cooling multiplier", true);
    settings.min_temperature = number(input, "minimum temperature");
    if (settings.cooling > 1 || settings.min_temperature >= settings.initial_temperature) {
        throw std::runtime_error("Annealing requires a cooling multiplier in (0,1] and Tmin < T0");
    }
    expect(input, "ADAPT");
    settings.update_every = integer(input, "adaptation period", 1);
    settings.decay = number(input, "weight update coefficient");
    settings.noise = number(input, "repair noise coefficient");
    for (auto& reward : settings.rewards) reward = number(input, "operator reward", true);
    if (settings.decay > 1) throw std::runtime_error("Weight update coefficient must be in [0,1]");
    expect(input, "EXACT");
    settings.exact_every = integer(input, "exact neighbourhood period", 0);
    settings.exact_pool = integer(input, "exact neighbourhood maximum pool size", 1);
    settings.exact_ms = integer(input, "exact neighbourhood time budget (ms)", 1);
    if (settings.exact_pool > 16) throw std::runtime_error("Exact neighbourhood pool limit must be at most 16");
    expect(input, "END");
    return settings;
}

void write_result(std::ostream& out, const Instance& instance, const SearchResult& result) {
    const auto& e = result.evaluation;
    out << std::setprecision(14)
        << "{\n  \"objective\": " << e.total
        << ",\n  \"initial_objective\": " << result.initial_cost
        << ",\n  \"operating_cost\": " << e.operating_cost
        << ",\n  \"carbon_cost\": " << e.carbon_cost
        << ",\n  \"waiting_cost\": " << e.waiting_cost
        << ",\n  \"emissions_kg_co2e\": " << e.carbon_kg
        << ",\n  \"truck_distance_m\": " << e.truck_distance
        << ",\n  \"truck_fuel_l\": " << e.fuel_l
        << ",\n  \"drone_distance_m\": " << e.drone_distance
        << ",\n  \"drone_energy_kwh\": " << e.energy_kwh
        << ",\n  \"truck_wait_s\": " << e.wait_s
        << ",\n  \"iterations\": " << result.iterations
        << ",\n  \"exact_calls\": " << result.exact_calls
        << ",\n  \"exact_completed\": " << result.exact_complete
        << ",\n  \"exact_improvements\": " << result.exact_improvements
        << ",\n  \"elapsed_s\": " << result.elapsed_s
        << ",\n  \"truck_tour\": [0";
    for (const auto& stop : result.best) out << ", " << stop.node;
    out << ", 0],\n  \"sorties\": [";
    bool first = true;
    for (const auto& stop : result.best) {
        for (const auto& sortie : stop.sorties) {
            const auto flight = evaluate_sortie(instance, stop.node, sortie);
            if (!first) out << ',';
            first = false;
            out << "\n    {\"anchor\": " << stop.node << ", \"route\": [" << stop.node;
            for (int node : sortie) out << ", " << node;
            out << ", " << stop.node << "], \"energy_kwh\": " << flight.energy
                << ", \"duration_s\": " << flight.duration << '}';
        }
    }
    if (!first) out << '\n' << "  ";
    out << "]\n}\n";
}

}  // namespace

int main(int argc, char** argv) {
    const bool help = argc == 2 && std::string(argv[1]) == "--help";
    const bool plain = argc == 3 && std::string(argv[2]) == "--plain";
    if (help || (argc != 2 && !plain)) {
        std::cerr << "Usage: truck_drone <instance.txt> [--plain]\n";
        return help ? 0 : 2;
    }
    try {
        std::ifstream file(argv[1]);
        if (!file) throw std::runtime_error("Cannot open instance file");
        file.imbue(std::locale::classic());
        std::cout.imbue(std::locale::classic());
        const Instance in = read_instance(file);
        Settings settings = read_settings(file, static_cast<int>(in.customers.size()) - 1);
        if (plain) settings.exact_every = 0;
        const auto result = solve(in, settings);
        if (!result.evaluation.feasible) throw std::logic_error("The solver did not produce a feasible incumbent");
        write_result(std::cout, in, result);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
