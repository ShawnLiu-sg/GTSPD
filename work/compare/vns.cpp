#include "benchmark.hpp"

#include <algorithm>
#include <array>
#include <random>
#include <utility>

namespace routing {
namespace {

enum class Neighborhood {
    truck_reverse,
    truck_to_drone,
    drone_to_truck,
    cross_stop_exchange,
    reanchor,
    sortie_reverse
};

constexpr std::array neighborhoods{
    Neighborhood::truck_reverse, Neighborhood::truck_to_drone, Neighborhood::drone_to_truck,
    Neighborhood::cross_stop_exchange, Neighborhood::reanchor, Neighborhood::sortie_reverse
};

struct Position {
    std::size_t stop{}, sortie{}, customer{};
};

struct Move {
    Neighborhood neighborhood;
    Position from, to;
};

template <typename Visitor>
bool visit_moves(const Instance& in, const Plan& plan, Neighborhood neighborhood,
                 Clock::time_point deadline, const Visitor& visit) {
    const auto offer = [&](Position from, Position to) {
        return Clock::now() < deadline && visit(Move{neighborhood, from, to});
    };
    const auto slots = static_cast<std::size_t>(in.drones);
    for (std::size_t i = 0; i < plan.size(); ++i) {
        if (Clock::now() >= deadline) return false;
        const auto& stop = plan[i];
        switch (neighborhood) {
        case Neighborhood::truck_reverse:
            for (std::size_t j = i + 1; j < plan.size(); ++j) {
                if (!offer({i}, {j})) return false;
            }
            break;
        case Neighborhood::truck_to_drone:
            if (!stop.sorties.empty() || !in.customers[stop.node].drone_eligible) break;
            for (std::size_t j = 0; j < plan.size(); ++j) {
                if (j == i || !in.customers[plan[j].node].launch_allowed) continue;
                const auto& target = plan[j].sorties;
                for (std::size_t r = 0; r < target.size(); ++r) {
                    for (std::size_t p = 0; p <= target[r].size(); ++p) {
                        if (!offer({i}, {j, r, p})) return false;
                    }
                }
                if (target.size() < slots && !offer({i}, {j, target.size()})) return false;
            }
            break;
        case Neighborhood::drone_to_truck:
            for (std::size_t r = 0; r < stop.sorties.size(); ++r) {
                for (std::size_t p = 0; p < stop.sorties[r].size(); ++p) {
                    for (std::size_t j = 0; j <= plan.size(); ++j) {
                        if (!offer({i, r, p}, {j})) return false;
                    }
                }
            }
            break;
        case Neighborhood::cross_stop_exchange:
            if (stop.sorties.empty()) break;
            for (std::size_t j = i + 1; j < plan.size(); ++j) {
                for (std::size_t r = 0; r < stop.sorties.size(); ++r) {
                    for (std::size_t s = 0; s < plan[j].sorties.size(); ++s) {
                        for (std::size_t p = 0; p < stop.sorties[r].size(); ++p) {
                            for (std::size_t q = 0; q < plan[j].sorties[s].size(); ++q) {
                                if (!offer({i, r, p}, {j, s, q})) return false;
                            }
                        }
                    }
                }
            }
            break;
        case Neighborhood::reanchor:
            if (stop.sorties.empty()) break;
            for (std::size_t j = 0; j < plan.size(); ++j) {
                if (j == i || !in.customers[plan[j].node].launch_allowed ||
                    plan[j].sorties.size() >= slots) continue;
                for (std::size_t r = 0; r < stop.sorties.size(); ++r) {
                    if (!offer({i, r}, {j})) return false;
                }
            }
            break;
        case Neighborhood::sortie_reverse:
            for (std::size_t r = 0; r < stop.sorties.size(); ++r) {
                for (std::size_t p = 0; p < stop.sorties[r].size(); ++p) {
                    for (std::size_t q = p + 1; q < stop.sorties[r].size(); ++q) {
                        if (!offer({i, r, p}, {i, r, q})) return false;
                    }
                }
            }
            break;
        }
    }
    return true;
}

Plan apply_move(Plan plan, const Move& move) {
    const auto& from = move.from;
    const auto& to = move.to;
    switch (move.neighborhood) {
    case Neighborhood::truck_reverse:
        std::reverse(plan.begin() + from.stop, plan.begin() + to.stop + 1);
        break;
    case Neighborhood::truck_to_drone: {
        auto& target = plan[to.stop].sorties;
        const int customer = plan[from.stop].node;
        if (to.sortie == target.size()) {
            target.push_back({customer});
        } else {
            auto& sortie = target[to.sortie];
            sortie.insert(sortie.begin() + to.customer, customer);
        }
        plan.erase(plan.begin() + from.stop);
        break;
    }
    case Neighborhood::drone_to_truck: {
        auto& source = plan[from.stop].sorties;
        auto& sortie = source[from.sortie];
        const int customer = sortie[from.customer];
        sortie.erase(sortie.begin() + from.customer);
        if (sortie.empty()) source.erase(source.begin() + from.sortie);
        plan.insert(plan.begin() + to.stop, Stop{customer, {}});
        break;
    }
    case Neighborhood::cross_stop_exchange:
        std::swap(plan[from.stop].sorties[from.sortie][from.customer],
                  plan[to.stop].sorties[to.sortie][to.customer]);
        break;
    case Neighborhood::reanchor: {
        auto& source = plan[from.stop].sorties;
        plan[to.stop].sorties.push_back(std::move(source[from.sortie]));
        source.erase(source.begin() + from.sortie);
        break;
    }
    case Neighborhood::sortie_reverse: {
        auto& sortie = plan[from.stop].sorties[from.sortie];
        std::reverse(sortie.begin() + from.customer, sortie.begin() + to.customer + 1);
        break;
    }
    }
    return plan;
}

struct Candidate {
    Plan plan;
    Evaluation evaluation;
};

Candidate first_improvement(const Instance& in, const Plan& current, const Evaluation& current_eval,
                            Neighborhood neighborhood, Clock::time_point deadline) {
    Candidate next;
    visit_moves(in, current, neighborhood, deadline, [&](const Move& move) {
        auto plan = apply_move(current, move);
        const auto evaluation = evaluate(in, plan);
        if (Clock::now() >= deadline) return false;
        if (evaluation.feasible && evaluation.total < current_eval.total) {
            next = {std::move(plan), evaluation};
            return false;
        }
        return true;
    });
    return next;
}

Candidate random_neighbor(const Instance& in, const Plan& current, std::mt19937_64& rng,
                          Clock::time_point deadline) {
    std::vector<Move> moves;
    for (const auto neighborhood : neighborhoods) {
        if (!visit_moves(in, current, neighborhood, deadline, [&](const Move& move) {
            moves.push_back(move);
            return true;
        })) return {};
    }
    std::shuffle(moves.begin(), moves.end(), rng);
    for (const auto& move : moves) {
        if (Clock::now() >= deadline) break;
        auto plan = apply_move(current, move);
        const auto evaluation = evaluate(in, plan);
        if (evaluation.feasible && Clock::now() < deadline) {
            return {std::move(plan), evaluation};
        }
    }
    return {};
}

}  // namespace

SearchResult solve_vns(const Instance& in, const Settings& settings, int shake_moves,
                       const IncumbentSink& incumbent) {
    const auto start = Clock::now();
    const auto deadline = start + std::chrono::milliseconds(settings.wall_ms);
    std::mt19937_64 rng(settings.seed);
    auto current = make_initial_plan(in, deadline);
    auto current_eval = evaluate(in, current);

    const auto record = [&](const Plan& plan, const Evaluation& evaluation) {
        if (incumbent) {
            incumbent(plan, evaluation,
                      std::chrono::duration<double, std::milli>(Clock::now() - start).count());
        }
    };
    SearchResult result;
    result.best = current;
    result.evaluation = current_eval;
    result.initial_cost = current_eval.total;
    if (current_eval.feasible) record(current, current_eval);

    const auto accept = [&](Candidate candidate) {
        current = std::move(candidate.plan);
        current_eval = candidate.evaluation;
        if (current_eval.total < result.evaluation.total) {
            result.best = current;
            result.evaluation = current_eval;
            record(current, current_eval);
        }
    };
    std::size_t k = 0;
    while (current_eval.feasible && Clock::now() < deadline) {
        ++result.iterations;
        if (k < neighborhoods.size()) {
            auto next = first_improvement(in, current, current_eval, neighborhoods[k], deadline);
            if (next.evaluation.feasible) {
                accept(std::move(next));
                k = 0;
            } else {
                ++k;
            }
            continue;
        }

        int moves = 0;
        for (; moves < shake_moves; ++moves) {
            auto next = random_neighbor(in, current, rng, deadline);
            if (!next.evaluation.feasible) break;
            accept(std::move(next));
        }
        if (moves == 0) break;
        k = 0;
    }
    result.elapsed_s = std::chrono::duration<double>(Clock::now() - start).count();
    return result;
}

}  // namespace routing
