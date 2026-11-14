#include "validation.hpp"

#include "gurobi_c.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <utility>

namespace validation {
namespace {

void require(int code, GRBenv* environment) {
    if (code != 0) throw std::runtime_error(GRBgeterrormsg(environment));
}

using Env = std::unique_ptr<GRBenv, decltype(&GRBfreeenv)>;
using Model = std::unique_ptr<GRBmodel, decltype(&GRBfreemodel)>;
using Coefficients = std::vector<std::pair<int,double>>;

Env open_environment() {
    GRBenv* env = nullptr;
    require(GRBemptyenv(&env), env);
    Env owned(env, GRBfreeenv);
    require(GRBsetintparam(env, GRB_INT_PAR_OUTPUTFLAG, 0), env);
    require(GRBstartenv(env), env);
    return owned;
}

class Builder {
public:
    explicit Builder(GRBenv* environment, const char* name) : model(nullptr, GRBfreemodel) {
        GRBmodel* pointer = nullptr;
        require(GRBnewmodel(environment, &pointer, name, 0, nullptr, nullptr, nullptr, nullptr, nullptr), environment);
        model.reset(pointer);
        env = GRBgetenv(pointer);
    }

    int variable(char type, double upper, double cost = 0, double lower = 0) {
        require(GRBaddvar(model.get(), 0, nullptr, nullptr, cost, lower, upper, type, nullptr), env);
        return vars++;
    }

    void row(const Coefficients& expression, char sense, double rhs) {
        std::map<int,double> coefficients;
        for (auto [col, coefficient] : expression) coefficients[col] += coefficient;
        std::vector<int> indices;
        std::vector<double> values;
        for (auto [col, coefficient] : coefficients) {
            if (coefficient == 0) continue;
            indices.push_back(col);
            values.push_back(coefficient);
        }
        require(GRBaddconstr(model.get(), static_cast<int>(indices.size()), indices.data(), values.data(),
                             sense, rhs, nullptr), env);
        ++rows;
    }

    int int_attr(const char* name) {
        int value = 0;
        require(GRBgetintattr(model.get(), name, &value), env);
        return value;
    }

    double real_attr(const char* name) {
        double value = 0;
        require(GRBgetdblattr(model.get(), name, &value), env);
        return value;
    }

    void solve(double seconds) {
        require(GRBsetdblparam(env, GRB_DBL_PAR_TIMELIMIT, seconds), env);
        require(GRBsetintparam(env, GRB_INT_PAR_THREADS, 1), env);
        require(GRBsetdblparam(env, GRB_DBL_PAR_MIPGAP, 0), env);
        require(GRBsetdblparam(env, GRB_DBL_PAR_MIPGAPABS, 1e-8), env);
        require(GRBsetdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, 1e-9), env);
        require(GRBsetdblparam(env, GRB_DBL_PAR_INTFEASTOL, 1e-9), env);
        require(GRBoptimize(model.get()), env);
    }

    Model model;
    GRBenv* env = nullptr;
    int vars = 0, rows = 0;
};

double remaining(Clock::time_point deadline) {
    return std::max(0.0, std::chrono::duration<double>(deadline - Clock::now()).count());
}

std::vector<int> eligible_customers(const Instance& instance) {
    std::vector<int> eligible;
    const int n = static_cast<int>(instance.customers.size());
    for (int i = 1; i < n; ++i) {
        if (instance.customers[i].drone_eligible && instance.customers[i].mass <= instance.drone_capacity) {
            eligible.push_back(i);
        }
    }
    return eligible;
}

int packing_bound(GRBenv* env, const Instance& in, const std::vector<int>& eligible, double seconds) {
    const int n = static_cast<int>(in.customers.size()) - 1;
    const int mandatory = n - static_cast<int>(eligible.size());
    if (in.drones == 0) return n;
    if (eligible.empty()) return mandatory;
    Builder packing(env, "packing_relaxation");
    std::vector<int> visited(n + 1);
    for (int i = 1; i <= n; ++i) visited[i] = packing.variable(GRB_BINARY, 1, 1);
    std::vector<Coefficients> served(n + 1);
    for (int i = 1; i <= n; ++i) served[i].push_back({visited[i], 1});
    for (int v = 1; v <= n; ++v) {
        for (int d = 0; d < in.drones; ++d) {
            Coefficients capacity{{visited[v], -in.drone_capacity}};
            for (int i : eligible) {
                if (i == v) continue;
                const int assign = packing.variable(GRB_BINARY, 1);
                capacity.push_back({assign, in.customers[i].mass});
                served[i].push_back({assign, 1});
            }
            packing.row(capacity, GRB_LESS_EQUAL, 0);
        }
    }
    for (int i = 1; i <= n; ++i) packing.row(served[i], GRB_EQUAL, 1);
    packing.solve(seconds);
    const double bound = packing.real_attr(GRB_DBL_ATTR_OBJBOUND);
    if (!std::isfinite(bound) || bound < mandatory || bound > n) return std::max(1, mandatory);
    return std::max(1, static_cast<int>(std::ceil(bound - 1e-7)));
}

double leg_energy(const Instance& in, int from, int to, double mass) {
    return (in.drone_empty_w + in.drone_payload_w_per_kg * mass)
           * in.air[from][to] / (3600000.0 * in.drone_speed);
}

using Cut = std::pair<std::vector<std::pair<int,double>>, double>;

std::vector<Cut> capacity_covers(const Instance& in, const std::vector<int>& vertices) {
    std::vector<Cut> cuts;
    const int count = static_cast<int>(vertices.size());
    std::set<std::vector<int>> seen;
    for (int start = 0; start < count; ++start) {
        double mass = 0;
        std::vector<int> cover;
        for (int step = 0; step < count && mass <= in.drone_capacity + 1e-9; ++step) {
            const int node = vertices[(start + step) % count];
            cover.push_back(node);
            mass += in.customers[node].mass;
        }
        if (mass <= in.drone_capacity + 1e-9) break;
        for (auto it = cover.begin(); it != cover.end();) {
            if (mass - in.customers[*it].mass > in.drone_capacity + 1e-9) {
                mass -= in.customers[*it].mass;
                it = cover.erase(it);
            } else {
                ++it;
            }
        }
        std::sort(cover.begin(), cover.end());
        if (!seen.insert(cover).second) continue;
        const double rhs = static_cast<double>(cover.size() - 1);
        std::vector<std::pair<int,double>> terms;
        for (int node : cover) terms.push_back({node,1});
        // Exact sequential lifting on a bounded seed. Remaining zero coefficients
        // retain validity without solving an exponential full-candidate knapsack.
        for (int node : vertices) {
            if (terms.size() >= 14) break;
            if (std::binary_search(cover.begin(), cover.end(), node)) continue;
            const double residual = in.drone_capacity - in.customers[node].mass;
            double best = 0;
            const auto knapsack = [&](auto&& recurse, std::size_t i, double weight, double score) -> void {
                if (weight > residual + 1e-9) return;
                if (i == terms.size()) {
                    best = std::max(best, score);
                    return;
                }
                recurse(recurse, i + 1, weight, score);
                recurse(recurse, i + 1, weight + in.customers[terms[i].first].mass, score + terms[i].second);
            };
            knapsack(knapsack, 0, 0, 0);
            if (rhs > best) terms.push_back({node,rhs-best});
        }
        cuts.push_back({std::move(terms), rhs});
    }
    return cuts;
}

std::vector<Cut> energy_cuts(const Instance& in, int anchor, const std::vector<int>& vertices) {
    std::vector<Cut> cuts;
    const int n = static_cast<int>(in.customers.size());
    std::vector<bool> reachable(n, false);
    std::vector<std::vector<bool>> conflict(n, std::vector<bool>(n));
    for (int i : vertices) {
        const double energy = leg_energy(in, anchor, i, in.customers[i].mass) + leg_energy(in, i, anchor, 0);
        reachable[i] = energy <= in.drone_budget + 1e-9;
        if (!reachable[i]) cuts.push_back({{{i,1}},0});
    }
    std::vector<int> degree(n, 0);
    for (int i : vertices) {
        if (!reachable[i]) continue;
        for (int j : vertices) {
            if (j <= i || !reachable[j]) continue;
            const double mass_i = in.customers[i].mass, mass_j = in.customers[j].mass;
            const double both = mass_i + mass_j;
            const double first = leg_energy(in, anchor, i, both) + leg_energy(in, i, j, mass_j) + leg_energy(in, j, anchor, 0);
            const double second = leg_energy(in, anchor, j, both) + leg_energy(in, j, i, mass_i) + leg_energy(in, i, anchor, 0);
            if (both > in.drone_capacity + 1e-9 || std::min(first, second) > in.drone_budget + 1e-9) {
                cuts.push_back({{{i,1},{j,1}},1});
                conflict[i][j] = conflict[j][i] = true;
                ++degree[i];
                ++degree[j];
            }
        }
    }
    std::vector<int> ranked = vertices;
    std::sort(ranked.begin(), ranked.end(), [&](int i, int j) {
        return degree[i] != degree[j] ? degree[i] > degree[j] : i < j;
    });
    std::set<std::vector<int>> seen;
    for (int seed : ranked) {
        std::vector<int> clique{seed};
        for (int node : ranked) {
            if (std::all_of(clique.begin(), clique.end(), [&](int member) { return conflict[node][member]; })) {
                clique.push_back(node);
            }
        }
        if (clique.size() < 3) continue;
        std::sort(clique.begin(), clique.end());
        if (!seen.insert(clique).second) continue;
        Cut cut{{},1};
        for (int node : clique) cut.first.push_back({node,1});
        cuts.push_back(std::move(cut));
    }
    return cuts;
}

struct Arc { int used = -1, load = -1; };
using Arcs = std::vector<std::vector<Arc>>;
struct Drone { std::vector<int> nodes; Arcs arc; int launch = -1; std::vector<int> delivered; };
struct Columns {
    std::vector<int> visited, dwell;
    Arcs road;
    std::vector<std::vector<Drone>> flights;
};

Columns build(Builder& b, const Instance& in, CutSelection cuts, const std::vector<int>& eligible,
              int stop_bound) {
    const int n = static_cast<int>(in.customers.size());
    const double road_carbon = in.carbon_cost_per_kg * in.fuel_co2_per_l;
    const double air_carbon = in.carbon_cost_per_kg * in.electricity_co2_per_kwh;
    const double mass = std::accumulate(in.customers.begin(), in.customers.end(), 0.0,
                                       [](double a, const Customer& c) { return a+c.mass; });
    Columns col;
    col.visited.resize(n);
    col.dwell.resize(n);
    col.road.assign(n, std::vector<Arc>(n));
    col.flights.resize(n);
    std::vector<bool> allowed(n, false);
    for (int node : eligible) allowed[node] = true;
    for (int i = 1; i < n; ++i) {
        col.visited[i] = b.variable(GRB_BINARY, 1, 0, allowed[i] ? 0 : 1);
        col.dwell[i] = b.variable(GRB_CONTINUOUS, GRB_INFINITY, in.wait_cost_per_s);
    }
    std::vector<Coefficients> inflow(n), outflow(n), payload(n);
    Coefficients initial_mass;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            const double length = in.road[i][j];
            const double time = length/in.truck_speed;
            auto& arc = col.road[i][j];
            arc.used = b.variable(GRB_BINARY, 1, in.truck_cost_per_m*length + road_carbon*in.truck_empty_l_per_s*time);
            arc.load = b.variable(GRB_CONTINUOUS, j == 0 ? 0 : in.truck_capacity,
                                  road_carbon*in.truck_payload_l_per_s_kg*time);
            b.row({{arc.load,1},{arc.used,-in.truck_capacity}},GRB_LESS_EQUAL,0);
            inflow[j].push_back({arc.used,1});
            outflow[i].push_back({arc.used,1});
            payload[j].push_back({arc.load,1});
            payload[i].push_back({arc.load,-1});
            if (i == 0) initial_mass.push_back({arc.load,1});
        }
    }
    b.row(inflow[0],GRB_EQUAL,1);
    b.row(outflow[0],GRB_EQUAL,1);
    b.row(initial_mass,GRB_EQUAL,mass);
    std::vector<Coefficients> service(n);
    for (int i = 1; i < n; ++i) {
        inflow[i].push_back({col.visited[i],-1});
        outflow[i].push_back({col.visited[i],-1});
        b.row(inflow[i],GRB_EQUAL,0);
        b.row(outflow[i],GRB_EQUAL,0);
        payload[i].push_back({col.visited[i],-in.customers[i].mass});
        service[i].push_back({col.visited[i],1});
    }

    for (int v = 1; v < n; ++v) {
        if (!in.customers[v].launch_allowed || in.drones == 0) continue;
        std::vector<int> customers;
        for (int i : eligible) if (i != v) customers.push_back(i);
        if (customers.empty()) continue;
        std::vector<Cut> extras;
        if (cuts.cover) extras = capacity_covers(in, customers);
        if (cuts.energy) {
            auto battery = energy_cuts(in,v,customers);
            extras.insert(extras.end(), battery.begin(), battery.end());
        }
        std::vector<int> nodes{v};
        nodes.insert(nodes.end(), customers.begin(), customers.end());
        col.flights[v].resize(in.drones);
        Coefficients last_order;
        for (int d = 0; d < in.drones; ++d) {
            Drone& drone = col.flights[v][d];
            drone.nodes = nodes;
            drone.arc.assign(n,std::vector<Arc>(n));
            drone.launch = b.variable(GRB_BINARY,1);
            drone.delivered.assign(n,-1);
            b.row({{drone.launch,1},{col.visited[v],-1}},GRB_LESS_EQUAL,0);
            for (int i : customers) {
                const int z = b.variable(GRB_BINARY,1);
                drone.delivered[i] = z;
                service[i].push_back({z,1});
                payload[v].push_back({z,-in.customers[i].mass});
                b.row({{z,1},{drone.launch,-1}},GRB_LESS_EQUAL,0);
            }
            std::vector<Coefficients> in_arc(n), out_arc(n), flight_mass(n);
            Coefficients energy, time{{col.dwell[v],-1}}, capacity{{drone.launch,-in.drone_capacity}}, order;
            for (int i : nodes) {
                for (int j : nodes) {
                    if (i == j) continue;
                    const double length = in.air[i][j];
                    const double seconds = length/in.drone_speed;
                    const double base_kwh = in.drone_empty_w*seconds/3600000.0;
                    const double payload_kwh = in.drone_payload_w_per_kg*seconds/3600000.0;
                    Arc& arc = drone.arc[i][j];
                    arc.used = b.variable(GRB_BINARY,1,in.drone_cost_per_m*length + air_carbon*base_kwh);
                    arc.load = b.variable(GRB_CONTINUOUS,j == v ? 0 : in.drone_capacity,air_carbon*payload_kwh);
                    b.row({{arc.load,1},{arc.used,-in.drone_capacity}},GRB_LESS_EQUAL,0);
                    in_arc[j].push_back({arc.used,1});
                    out_arc[i].push_back({arc.used,1});
                    flight_mass[j].push_back({arc.load,1});
                    flight_mass[i].push_back({arc.load,-1});
                    energy.push_back({arc.used,base_kwh});
                    energy.push_back({arc.load,payload_kwh});
                    time.push_back({arc.used,seconds});
                    if (i == v) order.push_back({arc.used,static_cast<double>(j)});
                }
            }
            for (int i : nodes) {
                const int active = i == v ? drone.launch : drone.delivered[i];
                in_arc[i].push_back({active,-1});
                out_arc[i].push_back({active,-1});
                b.row(in_arc[i],GRB_EQUAL,0);
                b.row(out_arc[i],GRB_EQUAL,0);
                if (i != v) {
                    flight_mass[i].push_back({active,-in.customers[i].mass});
                    b.row(flight_mass[i],GRB_EQUAL,0);
                    capacity.push_back({active,in.customers[i].mass});
                }
            }
            b.row(energy,GRB_LESS_EQUAL,in.drone_budget);
            b.row(time,GRB_LESS_EQUAL,0);
            b.row(capacity,GRB_LESS_EQUAL,0);
            if (cuts.symmetry && d > 0) {
                Coefficients row = last_order;
                for (auto [index, value] : order) row.push_back({index,-value});
                b.row(row,GRB_GREATER_EQUAL,0);
            }
            last_order = std::move(order);
            for (const Cut& cut : extras) {
                Coefficients row{{drone.launch,-cut.second}};
                for (auto [index, value] : cut.first) row.push_back({drone.delivered[index],value});
                b.row(row,GRB_LESS_EQUAL,0);
            }
        }
    }
    for (int i = 1; i < n; ++i) {
        b.row(service[i],GRB_EQUAL,1);
        b.row(payload[i],GRB_EQUAL,0);
    }
    if (cuts.packing) {
        Coefficients visits;
        for (int i = 1; i < n; ++i) visits.push_back({col.visited[i],1});
        b.row(visits,GRB_GREATER_EQUAL,stop_bound);
    }
    return col;
}

void install_start(Builder& b, const Columns& col, const Instance& in, const Plan& all_truck) {
    std::vector<double> warm(b.vars,0);
    double mass = 0;
    for (const Stop& stop : all_truck) mass += in.customers[stop.node].mass;
    int previous = 0;
    for (const Stop& stop : all_truck) {
        warm[col.visited[stop.node]] = 1;
        warm[col.road[previous][stop.node].used] = 1;
        warm[col.road[previous][stop.node].load] = mass;
        mass -= in.customers[stop.node].mass;
        previous = stop.node;
    }
    warm[col.road[previous][0].used] = 1;
    require(GRBsetdblattrarray(b.model.get(),GRB_DBL_ATTR_START,0,b.vars,warm.data()),b.env);
}

Plan decode(const Columns& col, const std::vector<double>& solution) {
    Plan plan;
    const int n = static_cast<int>(col.road.size());
    int current = 0;
    const auto next = [&](const Arcs& arcs, int from) {
        for (int i = 0; i < n; ++i) {
            const int arc = arcs[from][i].used;
            if (arc >= 0 && solution[arc] > 0.5) return i;
        }
        return -1;
    };
    for (int step = 0; step < n; ++step) {
        current = next(col.road, current);
        if (current < 0) return {};
        if (current == 0) return plan;
        Stop stop{current,{}};
        for (const Drone& d : col.flights[current]) {
            if (solution[d.launch] <= 0.5) continue;
            Sortie sortie;
            int i = current;
            for (int j = 0; j < n; ++j) {
                i = next(d.arc,i);
                if (i < 0) return {};
                if (i == current) break;
                sortie.push_back(i);
            }
            if (i != current || sortie.empty()) return {};
            stop.sorties.push_back(std::move(sortie));
        }
        plan.push_back(std::move(stop));
    }
    return {};
}

int __stdcall capture_root(GRBmodel*, void* cbdata, int where, void* userdata) {
    if (where == GRB_CB_MIPNODE) {
        double nodes = 0, bound = 0;
        int status = 0;
        if (GRBcbget(cbdata,where,GRB_CB_MIPNODE_NODCNT,&nodes) == 0 && nodes == 0 &&
            GRBcbget(cbdata,where,GRB_CB_MIPNODE_STATUS,&status) == 0 && status == GRB_OPTIMAL &&
            GRBcbget(cbdata,where,GRB_CB_MIPNODE_OBJBND,&bound) == 0 && std::abs(bound) < GRB_INFINITY) {
            double& captured = *static_cast<double*>(userdata);
            captured = std::max(captured,bound);
        }
    }
    return 0;
}

}  // namespace

MilpResult solve_milp(const Instance& in, const Plan& start, CutSelection cuts, double seconds,
                      bool warm_start) {
    const auto begun = Clock::now();
    const auto deadline = begun + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    Env env = open_environment();
    const auto eligible = eligible_customers(in);
    MilpResult out;
    if (cuts.packing) out.packing_bound = packing_bound(env.get(),in,eligible,std::min(0.3,remaining(deadline)));
    Builder b(env.get(),"truck_drone_arc");
    const Columns columns = build(b,in,cuts,eligible,out.packing_bound);
    if (warm_start) install_start(b,columns,in,start);
    require(GRBsetcallbackfunc(b.model.get(),capture_root,&out.root_bound),b.env);
    b.solve(remaining(deadline));
    out.status = b.int_attr(GRB_INT_ATTR_STATUS);
    out.incumbents = b.int_attr(GRB_INT_ATTR_SOLCOUNT);
    out.variables = b.vars;
    out.constraints = b.rows;
    out.lower_bound = b.real_attr(GRB_DBL_ATTR_OBJBOUND);
    out.nodes = b.real_attr(GRB_DBL_ATTR_NODECOUNT);
    out.solver_s = b.real_attr(GRB_DBL_ATTR_RUNTIME);
    if (out.incumbents > 0) {
        out.objective = b.real_attr(GRB_DBL_ATTR_OBJVAL);
        std::vector<double> solution(b.vars);
        require(GRBgetdblattrarray(b.model.get(),GRB_DBL_ATTR_X,0,b.vars,solution.data()),b.env);
        out.plan = decode(columns,solution);
        const Evaluation verified = evaluate_forward(in,out.plan);
        out.decoded_wait_s = verified.wait_s;
        out.decoded_cost = verified.total;
        out.feasible = verified.feasible && std::abs(out.objective - out.decoded_cost) <= 1e-6;
        if (!out.feasible) {
            std::ostringstream detail;
            detail << std::setprecision(16) << "MILP incumbent failed independent forward plan verification: "
                   << "MILP=" << out.objective << ", decoded=" << out.decoded_cost << ", feasible=" << verified.feasible;
            for (const auto& stop : out.plan) {
                detail << " | stop " << stop.node;
                for (const auto& route : stop.sorties) {
                    detail << " [";
                    for (int node : route) detail << ' ' << node;
                    detail << ']';
                }
            }
            throw std::runtime_error(detail.str());
        }
    }
    out.elapsed_s = std::chrono::duration<double>(Clock::now()-begun).count();
    const auto lp_started = Clock::now();
    GRBmodel* lp_pointer = nullptr;
    require(GRBrelaxmodel(b.model.get(),&lp_pointer),b.env);
    Model lp(lp_pointer,GRBfreemodel);
    auto* lp_env = GRBgetenv(lp.get());
    require(GRBsetintparam(lp_env,GRB_INT_PAR_THREADS,1),lp_env);
    require(GRBsetdblparam(lp_env,GRB_DBL_PAR_TIMELIMIT,std::min(2.0,seconds)),lp_env);
    require(GRBoptimize(lp.get()),lp_env);
    require(GRBgetintattr(lp.get(),GRB_INT_ATTR_STATUS,&out.relaxation_status),lp_env);
    if (out.relaxation_status == GRB_OPTIMAL) {
        require(GRBgetdblattr(lp.get(),GRB_DBL_ATTR_OBJVAL,&out.relaxation_objective),lp_env);
    }
    out.relaxation_s = std::chrono::duration<double>(Clock::now()-lp_started).count();
    return out;
}

RelaxationResult solve_relaxation(const Instance& in, CutSelection cuts, double seconds) {
    const auto started = Clock::now();
    const auto deadline = started + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    Env env = open_environment();
    const auto eligible = eligible_customers(in);
    RelaxationResult out;
    if (cuts.packing) out.packing_bound = packing_bound(env.get(),in,eligible,std::min(0.3,remaining(deadline)));
    Builder b(env.get(),"truck_drone_arc_relaxation");
    build(b,in,cuts,eligible,out.packing_bound);
    out.variables = b.vars;
    out.constraints = b.rows;
    require(GRBupdatemodel(b.model.get()),b.env);
    GRBmodel* lp_pointer = nullptr;
    require(GRBrelaxmodel(b.model.get(),&lp_pointer),b.env);
    Model lp(lp_pointer,GRBfreemodel);
    auto* lp_env = GRBgetenv(lp.get());
    require(GRBsetintparam(lp_env,GRB_INT_PAR_THREADS,1),lp_env);
    require(GRBsetdblparam(lp_env,GRB_DBL_PAR_TIMELIMIT,remaining(deadline)),lp_env);
    require(GRBsetdblparam(lp_env,GRB_DBL_PAR_FEASIBILITYTOL,1e-9),lp_env);
    require(GRBoptimize(lp.get()),lp_env);
    require(GRBgetintattr(lp.get(),GRB_INT_ATTR_STATUS,&out.status),lp_env);
    if (out.status == GRB_OPTIMAL) {
        require(GRBgetdblattr(lp.get(),GRB_DBL_ATTR_OBJVAL,&out.objective),lp_env);
    }
    out.elapsed_s = std::chrono::duration<double>(Clock::now()-started).count();
    return out;
}

MilpResult solve_local_milp(const Instance& in, const Plan& start, const std::vector<int>& stops,
                           const std::vector<std::vector<LocalPattern>>& patterns,
                           unsigned pool, double seconds) {
    const auto started = Clock::now();
    Env env = open_environment();
    Builder b(env.get(),"independent_local_pattern_model");
    std::vector<Coefficients> cover(in.customers.size());
    std::vector<std::vector<int>> selected(stops.size());
    for (std::size_t j = 0; j < stops.size(); ++j) {
        const int dwell = b.variable(GRB_CONTINUOUS,GRB_INFINITY,in.wait_cost_per_s);
        Coefficients fleet;
        for (const LocalPattern& p : patterns[j]) {
            const int x = b.variable(GRB_BINARY,1,p.cost);
            selected[j].push_back(x);
            fleet.push_back({x,1});
            b.row({{x,p.dwell},{dwell,-1}},GRB_LESS_EQUAL,0);
            for (std::size_t i = 1; i < cover.size(); ++i) {
                if (p.customers & (1u << i)) cover[i].push_back({x,1});
            }
        }
        b.row(fleet,GRB_LESS_EQUAL,in.drones);
    }
    for (std::size_t i = 1; i < cover.size(); ++i) {
        if (pool & (1u << i)) b.row(cover[i],GRB_EQUAL,1);
    }
    b.solve(seconds);
    MilpResult out;
    out.status = b.int_attr(GRB_INT_ATTR_STATUS);
    out.incumbents = b.int_attr(GRB_INT_ATTR_SOLCOUNT);
    out.variables = b.vars;
    out.constraints = b.rows;
    if (out.incumbents > 0) {
        out.objective = b.real_attr(GRB_DBL_ATTR_OBJVAL);
        std::vector<double> solution(b.vars);
        require(GRBgetdblattrarray(b.model.get(),GRB_DBL_ATTR_X,0,b.vars,solution.data()),b.env);
        out.plan = start;
        for (std::size_t j = 0; j < stops.size(); ++j) {
            auto& sorties = out.plan[stops[j]].sorties;
            sorties.clear();
            for (std::size_t i = 0; i < selected[j].size(); ++i) {
                if (solution[selected[j][i]] > 0.5) sorties.push_back(patterns[j][i].route);
            }
        }
        const auto check = evaluate_forward(in,out.plan);
        out.feasible = check.feasible;
        out.decoded_cost = check.total;
        out.decoded_wait_s = check.wait_s;
    }
    out.elapsed_s = std::chrono::duration<double>(Clock::now()-started).count();
    return out;
}

}  // namespace validation
