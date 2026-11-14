#include "benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <locale>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

using namespace routing;
namespace fs = std::filesystem;

enum class Method { hybrid, plain, fixed_q, nonadaptive, within_stop, order_first, vns };

const std::map<std::string, Method> method_names{
    {"hybrid", Method::hybrid}, {"plain", Method::plain}, {"fixed_q", Method::fixed_q},
    {"nonadaptive", Method::nonadaptive}, {"within_stop", Method::within_stop},
    {"order_first", Method::order_first}, {"vns", Method::vns}
};

struct Case {
    std::string id, stratum;
    Instance instance;
    Settings settings;
    std::optional<double> target;
};

struct Benchmark {
    int budget_ms{}, fixed_q{}, block_limit{}, perturb_moves{}, shake_moves{}, bootstrap{};
    std::uint64_t bootstrap_seed{};
    std::vector<int> checkpoints;
    std::vector<std::uint64_t> seeds;
    std::vector<std::string> methods;
    std::vector<Case> cases;
};

struct Observation {
    double elapsed_ms;
    double objective;
};

struct Trial {
    std::vector<Observation> history;
    Plan plan;
    Evaluation evaluation;
    SearchResult search;

    std::optional<double> at(int checkpoint) const {
        const auto after = std::upper_bound(history.begin(), history.end(), checkpoint,
            [](double time, const Observation& entry) { return time < entry.elapsed_ms; });
        if (after == history.begin()) return std::nullopt;
        return std::prev(after)->objective;
    }
};

using Trials = std::vector<std::vector<std::vector<Trial>>>;

template<class T>
T read(std::istream& input, const char* name) {
    T result{};
    if (!(input >> result)) throw std::runtime_error(std::string("Expected ") + name);
    return result;
}

void expect(std::istream& input, const char* tag) {
    if (read<std::string>(input, tag) != tag) throw std::runtime_error(std::string("Expected ") + tag);
}

int positive(std::istream& input, const char* name) {
    const int value = read<int>(input, name);
    if (value <= 0) throw std::runtime_error(std::string(name) + " must be positive");
    return value;
}

std::ifstream open_input(const fs::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot read " + path.string());
    stream.imbue(std::locale::classic());
    return stream;
}

Benchmark load_benchmark(const fs::path& path) {
    auto in = open_input(path);
    Benchmark b;
    expect(in, "BENCHMARK");
    expect(in, "1");
    expect(in, "BUDGET_MS");
    b.budget_ms = positive(in, "budget");
    expect(in, "CHECKPOINTS");
    const int checkpoint_count = positive(in, "checkpoint count");
    for (int i = 0; i < checkpoint_count; ++i) {
        const int time = positive(in, "checkpoint time");
        if (time > b.budget_ms || (!b.checkpoints.empty() && time <= b.checkpoints.back())) {
            throw std::runtime_error("Checkpoints must increase and fit within BUDGET_MS");
        }
        b.checkpoints.push_back(time);
    }
    if (b.checkpoints.back() < b.budget_ms) b.checkpoints.push_back(b.budget_ms);

    expect(in, "SEEDS");
    const int seed_count = positive(in, "seed count");
    std::set<std::uint64_t> seed_ids;
    for (int i = 0; i < seed_count; ++i) {
        const auto seed = read<std::uint64_t>(in, "seed");
        if (!seed_ids.insert(seed).second) throw std::runtime_error("Duplicate seed");
        b.seeds.push_back(seed);
    }
    expect(in, "METHODS");
    const int method_count = positive(in, "method count");
    std::set<std::string> methods;
    for (int i = 0; i < method_count; ++i) {
        const auto name = read<std::string>(in, "method name");
        if (method_names.count(name) == 0 || !methods.insert(name).second) {
            throw std::runtime_error("Unknown or duplicate method " + name);
        }
        b.methods.push_back(name);
    }
    expect(in, "FIXED_Q");
    b.fixed_q = positive(in, "fixed removal size");
    expect(in, "ORDER_FIRST");
    b.block_limit = positive(in, "maximum contiguous block size");
    if (b.block_limit > 16) throw std::runtime_error("Maximum contiguous block size is 16");
    b.perturb_moves = positive(in, "order perturbation moves");
    expect(in, "VNS");
    b.shake_moves = positive(in, "VNS shake moves");
    expect(in, "BOOTSTRAP");
    b.bootstrap = positive(in, "bootstrap replicates");
    b.bootstrap_seed = read<std::uint64_t>(in, "bootstrap seed");

    expect(in, "CASES");
    const int case_count = positive(in, "case count");
    std::set<std::string> ids;
    for (int i = 0; i < case_count; ++i) {
        Case c;
        c.id = read<std::string>(in, "case ID");
        if (!ids.insert(c.id).second) throw std::runtime_error("Duplicate case ID " + c.id);
        c.stratum = read<std::string>(in, "stratum");
        std::string file;
        if (!(in >> std::quoted(file))) throw std::runtime_error("Expected case path");
        const std::string target = read<std::string>(in, "frozen target or -");
        if (target != "-") {
            std::istringstream token(target);
            token.imbue(std::locale::classic());
            const double value = read<double>(token, "numeric target");
            if (token.peek() != std::char_traits<char>::eof() || !std::isfinite(value) || value < 0) {
                throw std::runtime_error("Invalid target for " + c.id);
            }
            c.target = value;
        }
        auto instance_file = open_input(path.parent_path() / fs::u8path(file));
        c.instance = read_instance(instance_file);
        const int customers = static_cast<int>(c.instance.customers.size()) - 1;
        c.settings = read_settings(instance_file, customers);
        if (methods.count("fixed_q") && b.fixed_q > customers) {
            throw std::runtime_error("FIXED_Q exceeds customer count for " + c.id);
        }
        if ((methods.count("hybrid") || methods.count("fixed_q") || methods.count("nonadaptive") ||
             methods.count("within_stop")) && c.settings.exact_every == 0) {
            throw std::runtime_error("EXACT period must be positive for active exact-neighbourhood methods in " + c.id);
        }
        b.cases.push_back(std::move(c));
    }
    expect(in, "END");
    return b;
}

Trial run_trial(const Benchmark& b, const Case& c, std::uint64_t seed, Method method) {
    Settings settings = c.settings;
    settings.seed = seed;
    settings.wall_ms = b.budget_ms;
    Trial run;
    const auto record = [&](const Plan& plan, const Evaluation& value, double time) {
        if (time <= b.budget_ms && value.feasible &&
            (run.history.empty() || value.total < run.history.back().objective)) {
            run.history.push_back({time, value.total});
            run.plan = plan;
            run.evaluation = value;
        }
    };

    if (method == Method::order_first) {
        run.search = solve_order_first(c.instance, settings, b.block_limit, b.perturb_moves, record);
    } else if (method == Method::vns) {
        run.search = solve_vns(c.instance, settings, b.shake_moves, record);
    } else {
        if (method == Method::plain) settings.exact_every = 0;
        if (method == Method::fixed_q) settings.q_min = settings.q_max = b.fixed_q;
        if (method == Method::nonadaptive) settings.decay = 0;
        run.search = solve_alns(c.instance, settings, method != Method::within_stop, record);
    }
    return run;
}

Trials run_benchmark(const Benchmark& b) {
    Trials trials(b.cases.size());
    for (std::size_t i = 0; i < b.cases.size(); ++i) {
        trials[i].resize(b.seeds.size());
        for (std::size_t s = 0; s < b.seeds.size(); ++s) {
            auto& pair = trials[i][s];
            pair.resize(b.methods.size());
            const std::size_t rotation = (i * b.seeds.size() + s) % b.methods.size();
            for (std::size_t k = 0; k < b.methods.size(); ++k) {
                const std::size_t m = (k + rotation) % b.methods.size();
                pair[m] = run_trial(b, b.cases[i], b.seeds[s], method_names.at(b.methods[m]));
            }
        }
    }
    return trials;
}

std::string csv(const std::string& value) {
    std::string quoted = "\"";
    for (char c : value) {
        quoted += c;
        if (c == '"') quoted += '"';
    }
    return quoted + '"';
}

std::ofstream open_output(const fs::path& dir, const char* name) {
    std::ofstream out(dir / name);
    if (!out) throw std::runtime_error(std::string("Cannot write ") + name);
    out.imbue(std::locale::classic());
    out << std::setprecision(15);
    return out;
}

std::optional<double> target_time(const Trial& run, const std::optional<double>& target) {
    if (!target) return std::nullopt;
    const auto hit = std::find_if(run.history.begin(), run.history.end(),
        [&](const Observation& entry) { return entry.objective <= *target; });
    if (hit == run.history.end()) return std::nullopt;
    return hit->elapsed_ms;
}

std::string tour_text(const Plan& plan) {
    std::ostringstream text;
    text << '0';
    for (const auto& stop : plan) text << ' ' << stop.node;
    text << " 0";
    return text.str();
}

std::string sorties_text(const Plan& plan) {
    std::ostringstream text;
    bool first = true;
    for (const auto& stop : plan) {
        for (const auto& sortie : stop.sorties) {
            if (!first) text << '|';
            first = false;
            text << stop.node << ':';
            for (std::size_t i = 0; i < sortie.size(); ++i) {
                if (i > 0) text << ' ';
                text << sortie[i];
            }
        }
    }
    return text.str();
}

void write_trials(const Benchmark& b, const Trials& trials, const fs::path& dir) {
    auto runs = open_output(dir, "runs.csv");
    auto checkpoints = open_output(dir, "checkpoints.csv");
    runs << "case,stratum,seed,method,budget_ms,status,objective,initial_objective,elapsed_ms,iterations,exact_calls,"
            "exact_completed,exact_improvements,operating_cost,carbon_cost,waiting_cost,emissions_kg_co2e,"
            "truck_distance_m,truck_fuel_l,drone_distance_m,drone_energy_kwh,truck_wait_s,target,target_status,"
            "target_time_ms,truck_tour,sorties\n";
    checkpoints << "case,stratum,seed,method,checkpoint_ms,status,objective\n";
    for (std::size_t i = 0; i < b.cases.size(); ++i) {
        const auto& c = b.cases[i];
        for (std::size_t s = 0; s < b.seeds.size(); ++s) {
            for (std::size_t m = 0; m < b.methods.size(); ++m) {
                const auto& t = trials[i][s][m];
                const bool valid = !t.history.empty();
                std::ostringstream id;
                id << csv(c.id) << ',' << csv(c.stratum) << ',' << b.seeds[s] << ',' << b.methods[m] << ',';
                const std::string prefix = id.str();
                const auto& e = t.evaluation;
                const auto& r = t.search;
                runs << prefix << b.budget_ms << ',' << (valid ? "ok" : "no_incumbent") << ',';
                if (valid) runs << e.total << ',' << r.initial_cost;
                else runs << ',';
                runs << ',' << r.elapsed_s * 1000.0 << ',' << r.iterations << ',' << r.exact_calls << ','
                     << r.exact_complete << ',' << r.exact_improvements << ',';
                if (valid) {
                    runs << e.operating_cost << ',' << e.carbon_cost << ',' << e.waiting_cost << ','
                         << e.carbon_kg << ',' << e.truck_distance << ',' << e.fuel_l << ','
                         << e.drone_distance << ',' << e.energy_kwh << ',' << e.wait_s;
                } else {
                    runs << ",,,,,,,,";
                }
                runs << ',';
                const auto hit = target_time(t, c.target);
                if (c.target) {
                    runs << *c.target << ',' << (hit ? "reached" : "right_censored") << ','
                         << hit.value_or(b.budget_ms);
                } else {
                    runs << ",disabled,";
                }
                runs << ',';
                if (valid) runs << csv(tour_text(t.plan)) << ',' << csv(sorties_text(t.plan));
                else runs << ',';
                runs << '\n';
                for (int time : b.checkpoints) {
                    const auto value = t.at(time);
                    checkpoints << prefix << time << ',' << (value ? "ok" : "no_incumbent") << ',';
                    if (value) checkpoints << *value;
                    checkpoints << '\n';
                }
            }
        }
    }
    if (!runs || !checkpoints) throw std::runtime_error("Cannot finish writing run files");
}

double mean(const std::vector<double>& values) {
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

void write_table7(const Benchmark& b, const Trials& trials, const fs::path& dir) {
    auto out = open_output(dir, "table7.csv");
    out << "case,stratum,method,budget_ms,runs,valid_runs,mean_objective,"
           "target,target_hits,target_censored,mean_time_up_to_budget_ms\n";
    for (std::size_t i = 0; i < b.cases.size(); ++i) {
        const auto& c = b.cases[i];
        for (std::size_t m = 0; m < b.methods.size(); ++m) {
            std::vector<double> costs, times;
            std::size_t hits = 0;
            for (const auto& seed : trials[i]) {
                const auto cost = seed[m].at(b.budget_ms);
                if (cost) costs.push_back(*cost);
                if (c.target) {
                    const auto hit = target_time(seed[m], c.target);
                    if (hit) ++hits;
                    times.push_back(hit.value_or(b.budget_ms));
                }
            }
            out << csv(c.id) << ',' << csv(c.stratum) << ',' << b.methods[m] << ',' << b.budget_ms
                << ',' << b.seeds.size() << ',' << costs.size() << ',';
            if (!costs.empty()) out << mean(costs);
            out << ',';
            if (c.target) {
                out << *c.target << ',' << hits << ',' << b.seeds.size() - hits << ',' << mean(times);
            } else {
                out << ",,,";
            }
            out << '\n';
        }
    }
    if (!out) throw std::runtime_error("Cannot finish writing table7.csv");
}

std::pair<double, double> bootstrap_interval(const std::vector<double>& effects, int samples,
                                             std::mt19937_64& rng) {
    std::uniform_int_distribution<std::size_t> index(0, effects.size() - 1);
    std::vector<double> draws;
    draws.reserve(samples);
    for (int i = 0; i < samples; ++i) {
        double sum = 0;
        for (std::size_t j = 0; j < effects.size(); ++j) sum += effects[index(rng)];
        draws.push_back(sum / static_cast<double>(effects.size()));
    }
    std::sort(draws.begin(), draws.end());
    const auto percentile = [&](double quantile) {
        const double position = quantile * static_cast<double>(draws.size() - 1);
        const auto lo = static_cast<std::size_t>(position);
        const auto hi = std::min(lo + 1, draws.size() - 1);
        return draws[lo] + (position - static_cast<double>(lo)) * (draws[hi] - draws[lo]);
    };
    return {percentile(0.025), percentile(0.975)};
}

void write_summary(const Benchmark& b, const Trials& trials, const fs::path& dir) {
    auto summary = open_output(dir, "summary.csv");
    auto paired = open_output(dir, "paired.csv");
    summary << "stratum,method,checkpoint_ms,instances,valid_instances,runs,valid_runs,mean_objective,"
               "sd_instance_objective,target_runs,target_hits,target_censored\n";
    paired << "stratum,method,checkpoint_ms,paired_instances,paired_runs,mean_delta,ci_low,ci_high\n";

    std::map<std::string, std::vector<std::size_t>> strata;
    for (std::size_t i = 0; i < b.cases.size(); ++i) strata[b.cases[i].stratum].push_back(i);
    const auto hybrid = static_cast<std::size_t>(std::find(b.methods.begin(), b.methods.end(), "hybrid") - b.methods.begin());
    std::mt19937_64 rng(b.bootstrap_seed);

    for (const auto& group : strata) {
        const auto& cases = group.second;
        for (std::size_t m = 0; m < b.methods.size(); ++m) {
            for (int time : b.checkpoints) {
                std::vector<double> instance_costs, paired_deltas;
                std::size_t valid_runs = 0, paired_runs = 0, target_runs = 0, hits = 0;
                for (std::size_t i : cases) {
                    std::vector<double> objectives, differences;
                    for (std::size_t s = 0; s < b.seeds.size(); ++s) {
                        const auto& run = trials[i][s][m];
                        if (time == b.budget_ms && b.cases[i].target) {
                            ++target_runs;
                            if (target_time(run, b.cases[i].target)) ++hits;
                        }
                        const auto objective = run.at(time);
                        if (objective) {
                            objectives.push_back(*objective);
                            if (hybrid < b.methods.size() && m != hybrid) {
                                const auto reference = trials[i][s][hybrid].at(time);
                                if (reference) differences.push_back(*objective - *reference);
                            }
                        }
                    }
                    valid_runs += objectives.size();
                    paired_runs += differences.size();
                    if (!objectives.empty()) instance_costs.push_back(mean(objectives));
                    if (!differences.empty()) paired_deltas.push_back(mean(differences));
                }
                summary << csv(group.first) << ',' << b.methods[m] << ',' << time << ',' << cases.size()
                        << ',' << instance_costs.size() << ',' << cases.size() * b.seeds.size()
                        << ',' << valid_runs << ',';
                if (!instance_costs.empty()) {
                    const double average = mean(instance_costs);
                    summary << average << ',';
                    if (instance_costs.size() > 1) {
                        double squares = 0;
                        for (double value : instance_costs) squares += (value - average) * (value - average);
                        summary << std::sqrt(squares / static_cast<double>(instance_costs.size() - 1));
                    }
                } else {
                    summary << ',';
                }
                summary << ',';
                if (time == b.budget_ms) summary << target_runs << ',' << hits << ',' << target_runs - hits;
                else summary << ",,";
                summary << '\n';

                if (hybrid < b.methods.size() && m != hybrid) {
                    paired << csv(group.first) << ',' << b.methods[m] << ',' << time << ','
                           << paired_deltas.size() << ',' << paired_runs << ',';
                    if (!paired_deltas.empty()) paired << mean(paired_deltas);
                    paired << ',';
                    if (paired_deltas.size() > 1) {
                        const auto [lo, hi] = bootstrap_interval(paired_deltas, b.bootstrap, rng);
                        paired << lo << ',' << hi;
                    } else {
                        paired << ',';
                    }
                    paired << '\n';
                }
            }
        }
    }
    if (!summary || !paired) throw std::runtime_error("Cannot finish writing summary files");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: truck_drone_compare <manifest.txt> <output_directory>\n";
        return 2;
    }
    try {
        const Benchmark b = load_benchmark(fs::path(argv[1]));
        const fs::path directory(argv[2]);
        fs::create_directories(directory);
        const Trials trials = run_benchmark(b);
        write_trials(b, trials, directory);
        write_table7(b, trials, directory);
        write_summary(b, trials, directory);
        std::cout << b.cases.size() * b.seeds.size() * b.methods.size() << " runs written to "
                  << directory.string() << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
