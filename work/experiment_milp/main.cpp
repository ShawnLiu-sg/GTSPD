#include "validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace {
using namespace validation;
namespace fs = std::filesystem;

const CutSelection no_cuts{};
const CutSelection all_cuts{true,true,true,true};

struct Case { std::string name; Instance instance; };

std::vector<Case> tiny_cases() {
    std::vector<Case> result;
    std::mt19937 rng(410936);
    for (int trial = 0; trial < 24; ++trial) {
        const int n = 5 + trial % 4;
        Case c;
        c.name = "tiny_" + std::to_string(trial+1);
        Instance& in = c.instance;
        in.customers.resize(n+1);
        in.road.assign(n+1, std::vector<double>(n+1));
        in.air = in.road;
        in.drones = 1 + trial % 3;
        in.truck_capacity = 60;
        in.drone_capacity = 4.5;
        in.drone_budget = .14 + (trial%5)*.09;
        in.truck_speed = 11.5;
        in.drone_speed = 11.5;
        in.truck_empty_l_per_s = .0025;
        in.truck_payload_l_per_s_kg = .0000015;
        in.drone_empty_w = 1179.31;
        in.drone_payload_w_per_kg = 184.54;
        in.truck_cost_per_m = .005693;
        in.drone_cost_per_m = .000684;
        in.carbon_cost_per_kg = .264;
        in.wait_cost_per_s = .005;
        in.fuel_co2_per_l = 3.24;
        in.electricity_co2_per_kwh = .577;
        std::vector<std::array<int,3>> coordinates(n+1);
        for (int i = 1; i <= n; ++i) {
            const double mass = .5 + .25*(rng()%16);
            in.customers[i] = {i <= 2 ? 6.0 : mass, i > 2, trial%7 != 0 || i%3 != 0};
            coordinates[i] = {static_cast<int>(rng()%4500), static_cast<int>(rng()%4500), static_cast<int>(rng()%500)};
        }
        for (int i = 0; i <= n; ++i) {
            for (int j = 0; j <= n; ++j) {
                const auto& x = coordinates[i];
                const auto& y = coordinates[j];
                in.air[i][j] = std::hypot(x[0]-y[0],x[1]-y[1]);
                in.road[i][j] = std::abs(x[0]-y[0]) + std::abs(x[1]-y[1]);
                if (trial%4 == 1) {
                    in.air[i][j] += std::max(0,y[2]-x[2]);
                    in.road[i][j] += std::max(0,x[2]-y[2]);
                }
            }
        }
        if (trial == 0) { in.drones = 0; c.name += "_zero_drones"; }
        if (trial == 1) { in.wait_cost_per_s = 0; c.name += "_zero_wait_price"; }
        if (trial == 2) { in.carbon_cost_per_kg = 0; c.name += "_zero_carbon_price"; }
        if (trial == 3) {
            for (int i = 1; i <= n; ++i) in.customers[i].launch_allowed = false;
            c.name += "_no_launch";
        }
        if (trial == 4) {
            in.truck_capacity = std::accumulate(in.customers.begin(),in.customers.end(),0.0,
                                               [](double total,const Customer& o) { return total+o.mass; });
            c.name += "_truck_at_capacity";
        }
        if (trial == 5) { in.drone_budget = 1e-6; c.name += "_unreachable_air"; }
        if (trial == 6) { in.customers[3].mass = in.drone_capacity; c.name += "_parcel_at_air_capacity"; }
        if (trial == 7) {
            for (int i = 1; i <= n; ++i) in.customers[i].launch_allowed = i == 1;
            c.name += "_one_launch_site";
        }
        result.push_back(std::move(c));
    }
    return result;
}

std::string plan_text(const Plan& plan) {
    std::ostringstream route;
    for (const Stop& stop : plan) {
        route << stop.node << ':';
        for (const auto& sortie : stop.sorties) {
            route << '(';
            for (int node : sortie) route << node << ' ';
            route << ')';
        }
        route << '|';
    }
    return route.str();
}

double as_gap(const MilpResult& solution) {
    if (!solution.feasible || !std::isfinite(solution.lower_bound) || std::abs(solution.lower_bound) >= 1e99 ||
        solution.objective <= 0) return infinity;
    return std::max(0.0,100*(solution.objective-solution.lower_bound)/solution.objective);
}

void number(std::ostream& out,double value) {
    if (std::isfinite(value) && std::abs(value) < 1e99) out << value;
}

void write_row(std::ostream& csv, const std::string& name, const std::string& variant, int n, int drones,
               double budget, bool warm, const MilpResult& m, double start_cost, const OracleResult* oracle) {
    csv << name << ',' << variant << ',' << n << ',' << drones << ',' << budget << ',' << warm << ',';
    if (warm) number(csv,start_cost);
    csv << ',' << m.status << ',' << m.incumbents << ',' << m.feasible << ',';
    number(csv,m.objective); csv << ','; number(csv,m.lower_bound); csv << ','; number(csv,as_gap(m)); csv << ',';
    number(csv,m.root_bound); csv << ',' << m.relaxation_status << ','; number(csv,m.relaxation_objective);
    csv << ',' << m.relaxation_s << ',' << m.solver_s << ',' << m.elapsed_s << ',' << m.nodes << ','
        << m.variables << ',' << m.constraints << ',' << m.packing_bound << ',';
    number(csv,m.decoded_cost); csv << ',';
    if (m.feasible) number(csv,m.decoded_wait_s);
    csv << ',';
    if (oracle != nullptr) {
        csv << oracle->checked.total << ',' << oracle->checked.wait_s << ',' << oracle->truck_tours << ','
            << oracle->complete_assignments << ',';
        if (m.feasible) number(csv,std::abs(m.objective-oracle->checked.total));
    } else {
        csv << ",,,,";
    }
    csv << '\n';
    csv.flush();
}

void run_tiny(std::ostream& csv) {
    for (const Case& c : tiny_cases()) {
        const Instance& in = c.instance;
        const OracleResult oracle = exhaustive(in);
        const auto checked = evaluate(in,oracle.plan);
        if (!oracle.checked.feasible || !checked.feasible || std::abs(checked.total-oracle.checked.total) > 1e-8) {
            throw std::runtime_error("Independent oracle failed cross-evaluation: "+c.name);
        }
        const Plan start = make_initial_plan(in,Clock::now()+std::chrono::seconds(1));
        const double start_cost = evaluate_forward(in,start).total;
        const std::vector<std::pair<std::string,CutSelection>> settings{{"base",no_cuts},{"full",all_cuts}};
        for (const auto& [variant,cuts] : settings) {
            constexpr double budget = 15;
            const auto m = solve_milp(in,start,cuts,budget,true);
            write_row(csv,c.name,variant,static_cast<int>(in.customers.size())-1,in.drones,budget,true,m,start_cost,&oracle);
            std::cout << c.name << ' ' << variant << ' ' << m.status << ' ' << m.objective << " vs " << oracle.checked.total << std::endl;
            if (m.status != 2 || std::abs(m.objective-oracle.checked.total) > 1e-6) {
                throw std::runtime_error("Tiny MILP did not certify oracle optimum: "+c.name);
            }
        }
    }
}

void run_tiny_search(std::ostream& csv) {
    csv << std::setprecision(16)
        << "case,customers,drones,seed,budget_ms,initial_cost,enumeration_objective,alns_objective,alns_gap_pct,"
        << "exact_calls,exact_complete,exact_improvements,iterations,elapsed_s,verified,enum_plan,alns_plan\n";
    for (const Case& c : tiny_cases()) {
        const OracleResult oracle = exhaustive(c.instance);
        Settings s;
        s.seed = 410936;
        s.iterations = 600;
        s.wall_ms = 100;
        s.q_min = 1;
        s.q_max = 4;
        s.sigmoid_alpha = 0.13;
        s.sigmoid_beta = 26;
        s.initial_temperature = 4;
        s.cooling = 0.993;
        s.min_temperature = 0.01;
        s.update_every = 20;
        s.decay = 0.3;
        s.noise = 0.08;
        s.rewards = {8,4,2,1};
        s.exact_every = 4;
        s.exact_pool = 8;
        s.exact_ms = 45;
        const SearchResult alns = solve_alns(c.instance,s,true,[](const Plan&,const Evaluation&,double) {});
        const Evaluation checked = evaluate_forward(c.instance,alns.best);
        const double gap = 100*(checked.total-oracle.checked.total)/oracle.checked.total;
        const bool good = checked.feasible && gap >= -1e-8 && std::abs(checked.total-alns.evaluation.total) <= 1e-8;
        csv << c.name << ',' << c.instance.customers.size()-1 << ',' << c.instance.drones << ',' << s.seed
            << ',' << s.wall_ms << ',' << alns.initial_cost << ',' << oracle.checked.total << ',' << checked.total
            << ',' << gap << ',' << alns.exact_calls << ',' << alns.exact_complete << ',' << alns.exact_improvements
            << ',' << alns.iterations << ',' << alns.elapsed_s << ',' << good << ',' << plan_text(oracle.plan)
            << ',' << plan_text(alns.best) << '\n';
        csv.flush();
        if (!good) throw std::runtime_error("ALNS objective or plan failed independent verification: "+c.name);
    }
}

Case local_case(int trial) {
    const int n = 5 + trial % 4;
    const std::string tag = trial == 0 ? "_tied_dwell" : trial == 1 ? "_unused_stop" :
                            trial == 2 ? "_air_cost_tradeoff" : trial == 3 ? "_launch_restriction" : "";
    Case c{"local_"+std::to_string(trial+1)+tag,{}};
    Instance& in = c.instance;
    in.customers.resize(n+1);
    in.road.assign(n+1,std::vector<double>(n+1));
    in.air = in.road;
    in.drones = trial == 0 || trial == 2 ? 3 : 2;
    in.truck_capacity = 60;
    in.drone_capacity = trial == 0 ? 1.5 : 3.5;
    in.drone_budget = .18 + .03*(trial%4);
    in.truck_speed = in.drone_speed = 12;
    in.truck_empty_l_per_s = .003;
    in.truck_payload_l_per_s_kg = trial == 2 ? .003 : .00001;
    in.drone_empty_w = 1250;
    in.drone_payload_w_per_kg = 170;
    in.truck_cost_per_m = .0057;
    in.drone_cost_per_m = .0009;
    in.carbon_cost_per_kg = trial == 2 ? 2.0 : .264;
    in.wait_cost_per_s = .005;
    in.fuel_co2_per_l = 3.24;
    in.electricity_co2_per_kwh = .577;
    std::vector<std::array<int,2>> coordinate(n+1);
    coordinate[0] = {0,0};
    coordinate[1] = {400,0};
    coordinate[2] = trial == 0 ? std::array<int,2>{400,0} : std::array<int,2>{1800,0};
    coordinate[3] = {1500,1800};
    for (int i = 1; i <= n; ++i) {
        in.customers[i] = {i <= 3 ? 5.0 : 1.0 + .125*((trial+i)%3),i > 3,i <= 2};
        if (i >= 4) coordinate[i] = {300 + (trial*613+i*347)%1600,200+(trial*211+i*433)%900};
    }
    if (trial == 0) {
        for (int i = 4; i <= n; ++i) { coordinate[i] = {900,0}; in.customers[i].mass = 1; }
    }
    if (trial == 2) {
        for (int i = 4; i <= n; ++i) coordinate[i] = {1700,(i-4)*20};
    }
    if (trial == 3) in.customers[2].launch_allowed = false;
    for (int i = 0; i <= n; ++i) for (int j = 0; j <= n; ++j) {
        const int dx = coordinate[i][0]-coordinate[j][0];
        const int dy = coordinate[i][1]-coordinate[j][1];
        in.road[i][j] = std::abs(dx)+std::abs(dy);
        in.air[i][j] = std::hypot(dx,dy);
        if (trial%4 == 1) in.air[i][j] += std::max(0,dy)/4.0;
    }
    return c;
}

void run_local(std::ostream& csv) {
    csv << std::setprecision(16)
        << "case,customers,selected_stops,pool,patterns,start_total,start_air_cost,enumeration_total,dp_total,milp_total,"
        << "dp_air_cost,dp_complete,milp_status,partition_count,dp_elapsed_s,milp_elapsed_s,verified,start_plan,dp_plan,milp_plan\n";
    for (int trial = 0; trial < 24; ++trial) {
        const Case c = local_case(trial);
        const Instance& in = c.instance;
        const int n = static_cast<int>(in.customers.size())-1;
        Plan initial{{1,{}},{2,{}},{3,{}}};
        for (int i = 4; i <= n; ++i) {
            const int position = trial == 0 || trial == 1 || trial == 3 ? 0 : trial == 2 ? 1 : (i-4)%2;
            auto& sorties = initial[position].sorties;
            if (static_cast<int>(sorties.size()) < in.drones) sorties.push_back({i});
            else sorties[(i-4)%in.drones].push_back(i);
        }
        const auto initial_value = evaluate_forward(in,initial);
        if (!initial_value.feasible) throw std::runtime_error("Bad local start: "+c.name);
        const std::vector<int> selected = trial%5 == 4 ? std::vector<int>{0} : std::vector<int>{0,1};
        unsigned pool = 0;
        for (int stop : selected) for (const auto& sortie : initial[stop].sorties) {
            for (int node : sortie) pool |= (1u << node);
        }
        std::vector<std::vector<LocalPattern>> patterns(selected.size());
        std::size_t pattern_count = 0;
        for (std::size_t v = 0; v < selected.size(); ++v) {
            const int anchor = initial[selected[v]].node;
            double prefix = 0;
            int previous = 0;
            for (int i = 0; i <= selected[v]; ++i) {
                prefix += in.road[previous][initial[i].node];
                previous = initial[i].node;
            }
            for (unsigned subset = pool; subset; subset = (subset-1)&pool) {
                Sortie route;
                double mass = 0;
                for (int i = 1; i <= n; ++i) if (subset & (1u << i)) { route.push_back(i); mass += in.customers[i].mass; }
                do {
                    const auto f = evaluate_flight_forward(in,anchor,route);
                    if (!f.feasible) continue;
                    const double fuel_cost = mass * prefix * in.truck_payload_l_per_s_kg / in.truck_speed
                                           * in.fuel_co2_per_l * in.carbon_cost_per_kg;
                    const double air_cost = f.distance * in.drone_cost_per_m
                                          + f.energy * in.electricity_co2_per_kwh * in.carbon_cost_per_kg;
                    patterns[v].push_back({route,subset,f.duration,fuel_cost+air_cost});
                    ++pattern_count;
                } while (std::next_permutation(route.begin(),route.end()));
            }
        }

        Plan plan = initial;
        for (int stop : selected) plan[stop].sorties.clear();
        double optimum = infinity;
        std::size_t partitions = 0;
        const auto allocate = [&](auto&& recurse,unsigned unserved) -> void {
            if (!unserved) {
                const auto e = evaluate_forward(in,plan);
                if (e.feasible) { optimum = std::min(optimum,e.total); ++partitions; }
                return;
            }
            const unsigned pivot = unserved & (0u-unserved);
            for (std::size_t v = 0; v < selected.size(); ++v) {
                auto& sorties = plan[selected[v]].sorties;
                if (static_cast<int>(sorties.size()) >= in.drones) continue;
                for (const auto& p : patterns[v]) {
                    if ((p.customers & unserved) != p.customers || !(p.customers & pivot)) continue;
                    sorties.push_back(p.route);
                    recurse(recurse,unserved^p.customers);
                    sorties.pop_back();
                }
            }
        };
        allocate(allocate,pool);

        Plan dp = initial;
        const auto start = Clock::now();
        const bool complete = improve_exact(in,dp,selected,start+std::chrono::seconds(5));
        const double dp_seconds = std::chrono::duration<double>(Clock::now()-start).count();
        const auto dp_value = evaluate_forward(in,dp);
        const auto milp = solve_local_milp(in,initial,selected,patterns,pool,5);
        const auto air_cost = [&](const Evaluation& e) {
            return e.drone_distance*in.drone_cost_per_m + e.energy_kwh*in.electricity_co2_per_kwh*in.carbon_cost_per_kg;
        };
        const bool good = complete && milp.status == 2 && milp.feasible && dp_value.feasible &&
                          std::abs(dp_value.total-optimum) <= 1e-7 && std::abs(milp.decoded_cost-optimum) <= 1e-7;
        int pool_count = 0;
        for (int i = 1; i <= n; ++i) if (pool & (1u << i)) ++pool_count;
        csv << c.name << ',' << n << ',' << selected.size() << ',' << pool_count << ',' << pattern_count
            << ',' << initial_value.total << ',' << air_cost(initial_value) << ',' << optimum << ',' << dp_value.total
            << ',' << milp.decoded_cost << ',' << air_cost(dp_value) << ',' << complete << ',' << milp.status << ','
            << partitions << ',' << dp_seconds << ',' << milp.elapsed_s << ',' << good << ','
            << plan_text(initial) << ',' << plan_text(dp) << ',' << plan_text(milp.plan) << '\n';
        csv.flush();
        if (!good) throw std::runtime_error("Local DP/MILP did not match exhaustive enumeration: "+c.name);
        if (trial == 2 && !(dp_value.total < initial_value.total && air_cost(dp_value) > air_cost(initial_value))) {
            throw std::runtime_error("The targeted air-cost trade-off did not materialize");
        }
    }
}

void run_s1(std::ostream& csv, const fs::path& cases, double budget, const std::string& phase) {
    std::vector<fs::path> inputs;
    for (const fs::directory_entry& entry : fs::directory_iterator(cases)) {
        const std::string name = entry.path().filename().string();
        if (entry.path().extension() == ".txt" && (name.rfind("n16_",0) == 0 || name.rfind("n25_",0) == 0)) {
            inputs.push_back(entry.path());
        }
    }
    std::sort(inputs.begin(),inputs.end());
    if (inputs.empty()) throw std::runtime_error("No S1 files were found");
    const bool warm = phase != "cold";
    for (std::size_t i = 0; i < inputs.size(); ++i) {
        const fs::path& path = inputs[i];
        const std::string name = path.stem().string();
        if (phase != "warm" && name.substr(name.size()-2) != "01" && name.substr(name.size()-2) != "02") continue;
        std::ifstream file(path);
        const Instance in = read_instance(file);
        const Plan start = make_initial_plan(in,Clock::now()+std::chrono::seconds(2));
        const auto check = evaluate_forward(in,start);
        if (!check.feasible) throw std::runtime_error("S1 start invalid: "+name);
        std::vector<std::pair<std::string,CutSelection>> variants;
        if (phase == "families") {
            variants = {{"packing_only",{true,false,false,false}},{"symmetry_only",{false,true,false,false}},
                        {"cover_only",{false,false,true,false}},{"energy_only",{false,false,false,true}},
                        {"without_packing",{false,true,true,true}},{"without_symmetry",{true,false,true,true}},
                        {"without_cover",{true,true,false,true}},{"without_energy",{true,true,true,false}}};
        } else if (i%2 == 0) {
            variants = {{"base",no_cuts},{"full",all_cuts}};
        } else {
            variants = {{"full",all_cuts},{"base",no_cuts}};
        }
        for (const auto& [variant,cuts] : variants) {
            const auto m = solve_milp(in,start,cuts,budget,warm);
            write_row(csv,name,variant,static_cast<int>(in.customers.size())-1,in.drones,budget,warm,m,check.total,nullptr);
            std::cout << name << ' ' << variant << ' ' << m.status << ' ' << m.objective << ' ' << m.lower_bound << std::endl;
        }
    }
}

void run_s1_relaxation(std::ostream& csv, const fs::path& cases, double budget) {
    csv << std::setprecision(16)
        << "case,variant,customers,budget_s,status,objective,elapsed_s,vars,rows,packing_bound\n";
    const std::vector<std::pair<std::string,CutSelection>> variants{{"base",no_cuts},{"full",all_cuts}};
    for (int size : {16,25}) {
        for (const std::string shape : {"U","R","CL"}) {
            const std::string name = "n"+std::to_string(size)+"_"+shape+"_01";
            std::ifstream file(cases/(name+".txt"));
            const Instance in = read_instance(file);
            for (const auto& [variant,cuts] : variants) {
                const auto lp = solve_relaxation(in,cuts,budget);
                csv << name << ',' << variant << ',' << size << ',' << budget << ',' << lp.status << ',';
                number(csv,lp.objective);
                csv << ',' << lp.elapsed_s << ',' << lp.variables << ',' << lp.constraints << ',' << lp.packing_bound << '\n';
                csv.flush();
                std::cout << name << ' ' << variant << " LP " << lp.status << ' ' << lp.objective << std::endl;
            }
        }
    }
}

}  // namespace

int main(int argc,char** argv) {
    try {
        if (argc != 3 && argc != 5 && argc != 6) throw std::runtime_error("usage: validation tiny|tiny-search|local out.csv | validation s1 out.csv case-dir seconds warm|cold|families | validation s1-lp out.csv case-dir seconds");
        const bool tiny_search = std::string(argv[1]) == "tiny-search" && argc == 3;
        const bool local = std::string(argv[1]) == "local" && argc == 3;
#ifdef _WIN32
        DWORD_PTR process_mask = 0, system_mask = 0;
        const int cpu_index = tiny_search || local ? 11 : 10;
        if (GetProcessAffinityMask(GetCurrentProcess(),&process_mask,&system_mask) && (process_mask & (DWORD_PTR{1} << cpu_index))) {
            if (!SetProcessAffinityMask(GetCurrentProcess(),DWORD_PTR{1} << cpu_index)) {
                throw std::runtime_error("Failed to set planned CPU affinity");
            }
            std::cout << "Process affinity: CPU " << cpu_index << '\n';
        } else {
            std::cout << "Planned CPU is unavailable; original affinity retained\n";
        }
#endif
        std::ofstream csv(argv[2]);
        if (!csv) throw std::runtime_error("Cannot open CSV destination");
        if (tiny_search) {
            run_tiny_search(csv);
            return 0;
        }
        if (local) {
            run_local(csv);
            return 0;
        }
        if (std::string(argv[1]) == "s1-lp" && argc == 5) {
            run_s1_relaxation(csv,argv[3],std::stod(argv[4]));
            return 0;
        }
        csv << std::setprecision(16)
            << "case,variant,customers,drones,budget_s,warm_start,start_cost,grb_status,incumbent_count,verified,objective,lower_bound,gap_pct,root_callback_bound,lp_relaxation_status,lp_relaxation_objective,lp_diagnostic_s,mip_solver_s,mip_elapsed_s,nodes,vars,rows,packing_bound,decoded_objective,decoded_wait_s,enumeration_objective,enumeration_wait_s,truck_tours,complete_assignments,abs_objective_diff\n";
        if (std::string(argv[1]) == "tiny" && argc == 3) run_tiny(csv);
        else if (std::string(argv[1]) == "s1" && argc == 6) run_s1(csv,argv[3],std::stod(argv[4]),argv[5]);
        else throw std::runtime_error("Invalid arguments");
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
