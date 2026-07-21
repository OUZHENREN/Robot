#include "cs625_kinematics/ik_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "cs625_kinematics/config_classifier.hpp"
#include "cs625_kinematics/fk_cs625.hpp"

namespace cs625_kinematics {

namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr std::array<double, 6> kJointMin = {
    -2.0 * kPi,
    -2.0 * kPi,
    -1.0 * kPi,
    -2.0 * kPi,
    -2.0 * kPi,
    -2.0 * kPi
};

constexpr std::array<double, 6> kJointMax = {
     2.0 * kPi,
     2.0 * kPi,
     1.0 * kPi,
     2.0 * kPi,
     2.0 * kPi,
     2.0 * kPi
};

constexpr double kPosTol = 1e-4;
constexpr double kRotTol = 1e-3;
constexpr int kMaxIterations = 150;
constexpr double kFiniteDiffEps = 1e-6;
constexpr double kDamping = 1e-4;
constexpr double kStepClamp = 0.35;

double normalizeAngle(double q)
{
    while (q > kPi) {
        q -= 2.0 * kPi;
    }
    while (q < -kPi) {
        q += 2.0 * kPi;
    }
    return q;
}

double shortestAngularDistance(double from, double to)
{
    return normalizeAngle(to - from);
}

Eigen::Vector3d rotationVectorFromMatrix(const Eigen::Matrix3d& R)
{
    Eigen::AngleAxisd aa(R);
    if (!std::isfinite(aa.angle()) || aa.axis().hasNaN()) {
        return Eigen::Vector3d::Zero();
    }
    return aa.axis() * aa.angle();
}

Eigen::Matrix<double, 6, 1> poseError6D(
    const Eigen::Matrix4d& current_T,
    const Eigen::Matrix4d& target_T)
{
    Eigen::Matrix<double, 6, 1> err;
    err.setZero();

    err.block<3,1>(0,0) =
        target_T.block<3,1>(0,3) - current_T.block<3,1>(0,3);

    const Eigen::Matrix3d Rc = current_T.block<3,3>(0,0);
    const Eigen::Matrix3d Rt = target_T.block<3,3>(0,0);

    const Eigen::Matrix3d Rerr = Rc.transpose() * Rt;
    err.block<3,1>(3,0) = rotationVectorFromMatrix(Rerr);

    return err;
}

Eigen::Matrix<double, 6, 6> numericalJacobian(
    const std::array<double, 6>& q)
{
    Eigen::Matrix<double, 6, 6> J;
    J.setZero();

    const Eigen::Matrix4d T0 = fk_cs625_ee(q);

    for (int i = 0; i < 6; ++i) {
        std::array<double, 6> q_eps = q;
        q_eps[i] += kFiniteDiffEps;

        const Eigen::Matrix4d T_eps = fk_cs625_ee(q_eps);
        const Eigen::Matrix<double, 6, 1> diff = poseError6D(T0, T_eps) / kFiniteDiffEps;
        J.col(i) = diff;
    }

    return J;
}

}  // namespace

IkSolver::IkSolver()
{
}

std::array<double,6> IkSolver::normalizeJoints(const std::array<double,6>& q) const
{
    std::array<double,6> out = q;

    for (size_t i = 0; i < 6; ++i) {
        out[i] = normalizeAngle(out[i]);
    }

    return out;
}

bool IkSolver::isWithinJointLimits(const std::array<double,6>& q) const
{
    constexpr double tol = 1e-9;
    for (size_t i = 0; i < 6; ++i) {
        if (q[i] < kJointMin[i] - tol || q[i] > kJointMax[i] + tol) {
            return false;
        }
    }
    return true;
}

bool IkSolver::validateSolution(
    const Eigen::Matrix4d& target_T,
    const std::array<double,6>& q_solution) const
{
    const Eigen::Matrix4d T_fk = fk_cs625_ee(q_solution);
    const double pos_err = positionError(T_fk, target_T);
    const double rot_err = orientationError(T_fk, target_T);

    return (pos_err <= kPosTol) && (rot_err <= kRotTol);
}

std::vector<std::array<double,6>> IkSolver::generateSeeds(
    const std::array<double,6>* seed,
    bool aggressive_search) const
{
    std::vector<std::array<double,6>> seeds;

    auto appendSeed = [&](const std::array<double,6>& s) {
        const std::array<double,6> n = normalizeJoints(s);

        for (const auto& existing : seeds) {
            if (isDuplicate(existing, n, 1e-6)) {
                return;
            }
        }
        seeds.push_back(n);
    };

    if (seed != nullptr) {
        appendSeed(*seed);

        // Around user/current seed, inject local perturbations to escape
        // the same local basin while preserving "nearby" preference.
        const std::array<double,6> base = normalizeJoints(*seed);

        const std::vector<double> dq1_set = {0.0, -kPi, kPi, -kPi * 0.5, kPi * 0.5};
        const std::vector<double> dq5_set = {0.0, -kPi, kPi, -kPi * 0.5, kPi * 0.5};
        const std::vector<double> dq3_set = {0.0, -1.2, 1.2};

        for (double dq1 : dq1_set) {
            for (double dq5 : dq5_set) {
                for (double dq3 : dq3_set) {
                    std::array<double,6> s = base;
                    s[0] += dq1;
                    s[2] += dq3;
                    s[4] += dq5;
                    appendSeed(s);
                }
            }
        }
    }

    // Conservative baseline seeds for single-solution search.
    appendSeed({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    appendSeed({0.0, -1.57, 1.57, 0.0, 0.0, 0.0});
    appendSeed({0.0, -1.20, 1.20, -1.57, 0.0, 0.0});
    appendSeed({0.0, -1.57, 1.57, 0.0,  kPi, 0.0});
    appendSeed({ kPi, -1.57, 1.57, 0.0, 0.0, 0.0});
    appendSeed({-kPi * 0.5, -1.0, 1.2, -1.2,  0.7, 0.0});
    appendSeed({ kPi * 0.5, -1.0, 1.2, -1.2, -0.7, 0.0});

    if (!aggressive_search) {
        return seeds;
    }

    // -------------------------------------------------------------------------
    // Aggressive seed expansion for solveIKAll():
    // Systematically span likely shoulder / elbow / wrist branches.
    // -------------------------------------------------------------------------

    const std::vector<double> q1_vals = {
        -kPi, -kPi * 0.5, 0.0, kPi * 0.5, kPi
    };

    const std::vector<double> q2_vals = {
        -2.4, -1.8, -1.2, -0.6, 0.6
    };

    const std::vector<double> q3_vals = {
        -2.4, -1.4, -0.6, 0.6, 1.4, 2.4
    };

    const std::vector<double> q4_vals = {
        -kPi, -kPi * 0.5, 0.0, kPi * 0.5, kPi
    };

    const std::vector<double> q5_vals = {
        -kPi, -kPi * 0.5, 0.0, kPi * 0.5, kPi
    };

    const std::vector<double> q6_vals = {
        -kPi, -kPi * 0.5, 0.0, kPi * 0.5, kPi
    };

    // Representative structured combinations rather than full cartesian product.
    for (double q1 : q1_vals) {
        for (double q2 : q2_vals) {
            for (double q3 : q3_vals) {
                appendSeed({q1, q2, q3, 0.0, 0.0, 0.0});
                appendSeed({q1, q2, q3, -kPi * 0.5, 0.0, 0.0});
                appendSeed({q1, q2, q3,  kPi * 0.5, 0.0, 0.0});

                appendSeed({q1, q2, q3, 0.0, -kPi * 0.5, 0.0});
                appendSeed({q1, q2, q3, 0.0,  kPi * 0.5, 0.0});

                appendSeed({q1, q2, q3, 0.0, 0.0, -kPi * 0.5});
                appendSeed({q1, q2, q3, 0.0, 0.0,  kPi * 0.5});

                appendSeed({q1, q2, q3, 0.0,  kPi, 0.0});
                appendSeed({q1, q2, q3, 0.0, -kPi, 0.0});
            }
        }
    }

    // Wrist-oriented seed sweep.
    for (double q4 : q4_vals) {
        for (double q5 : q5_vals) {
            for (double q6 : q6_vals) {
                appendSeed({0.0, -1.2,  1.2, q4, q5, q6});
                appendSeed({0.0, -1.8,  1.8, q4, q5, q6});
                appendSeed({kPi, -1.2,  1.2, q4, q5, q6});
                appendSeed({kPi, -1.8,  1.8, q4, q5, q6});
                appendSeed({-kPi * 0.5, -1.2, 1.2, q4, q5, q6});
                appendSeed({ kPi * 0.5, -1.2, 1.2, q4, q5, q6});
            }
        }
    }

    // Deterministic random restarts.
    // Fixed seed for reproducibility.
    std::mt19937 rng(6252024u);

    std::uniform_real_distribution<double> dist_q1(-kPi, kPi);
    std::uniform_real_distribution<double> dist_q2(-2.6, 1.0);
    std::uniform_real_distribution<double> dist_q3(-kPi, kPi);
    std::uniform_real_distribution<double> dist_q4(-kPi, kPi);
    std::uniform_real_distribution<double> dist_q5(-kPi, kPi);
    std::uniform_real_distribution<double> dist_q6(-kPi, kPi);

    constexpr int kRandomSeedCount = 80;
    for (int i = 0; i < kRandomSeedCount; ++i) {
        appendSeed({
            dist_q1(rng),
            dist_q2(rng),
            dist_q3(rng),
            dist_q4(rng),
            dist_q5(rng),
            dist_q6(rng)
        });
    }

    return seeds;
}

double IkSolver::solutionCost(
    const std::array<double,6>& q,
    const std::array<double,6>* reference) const
{
    if (reference == nullptr) {
        double sum = 0.0;
        for (double v : q) {
            sum += v * v;
        }
        return std::sqrt(sum);
    }

    double sum = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        const double d = shortestAngularDistance((*reference)[i], q[i]);
        sum += d * d;
    }
    return std::sqrt(sum);
}

bool IkSolver::isDuplicate(
    const std::array<double,6>& a,
    const std::array<double,6>& b,
    double tol) const
{
    for (size_t i = 0; i < 6; ++i) {
        const double d = std::abs(shortestAngularDistance(a[i], b[i]));
        if (d > tol) {
            return false;
        }
    }
    return true;
}

bool IkSolver::solveIKNumerical(
    const Eigen::Matrix4d& target_T,
    const std::array<double,6>& seed,
    std::array<double,6>& q_solution)
{
    std::array<double,6> q = normalizeJoints(seed);

    for (int iter = 0; iter < kMaxIterations; ++iter) {
        const Eigen::Matrix4d T_cur = fk_cs625_ee(q);
        const Eigen::Matrix<double, 6, 1> err = poseError6D(T_cur, target_T);

        const double pos_err = err.block<3,1>(0,0).norm();
        const double rot_err = err.block<3,1>(3,0).norm();

        if (pos_err <= kPosTol && rot_err <= kRotTol) {
            q_solution = normalizeJoints(q);
            return isWithinJointLimits(q_solution);
        }

        const Eigen::Matrix<double, 6, 6> J = numericalJacobian(q);
        const Eigen::Matrix<double, 6, 6> A =
            J.transpose() * J + kDamping * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 1> b = J.transpose() * err;
        Eigen::Matrix<double, 6, 1> dq = A.ldlt().solve(b);

        for (int i = 0; i < 6; ++i) {
            dq(i) = std::clamp(dq(i), -kStepClamp, kStepClamp);
            q[i] += dq(i);
        }

        q = normalizeJoints(q);

        for (size_t i = 0; i < 6; ++i) {
            q[i] = std::clamp(q[i], kJointMin[i], kJointMax[i]);
        }
    }

    q_solution = normalizeJoints(q);
    return false;
}

bool IkSolver::solveIK(
    const Eigen::Matrix4d& target_T,
    const std::array<double,6>* seed,
    std::array<double,6>& q_solution)
{
    std::vector<std::array<double,6>> seeds = generateSeeds(seed, false);

    double best_cost = std::numeric_limits<double>::infinity();
    bool found = false;
    std::array<double,6> best_q{};

    for (const auto& s : seeds) {
        std::array<double,6> q{};
        if (!solveIKNumerical(target_T, s, q)) {
            continue;
        }

        q = normalizeJoints(q);

        if (!isWithinJointLimits(q)) {
            continue;
        }

        if (!validateSolution(target_T, q)) {
            continue;
        }

        const double cost = solutionCost(q, seed);
        if (!found || cost < best_cost) {
            found = true;
            best_cost = cost;
            best_q = q;
        }
    }

    if (!found) {
        return false;
    }

    q_solution = best_q;
    return true;
}

bool IkSolver::solveIKAll(
    const Eigen::Matrix4d& target_T,
    int max_solutions,
    const std::array<double,6>* seed,
    std::vector<IkSolutionInternal>& solutions)
{
    solutions.clear();

    if (max_solutions <= 0) {
        return false;
    }

    const std::vector<std::array<double,6>> seeds = generateSeeds(seed, true);

    for (const auto& s : seeds) {
        std::array<double,6> q{};
        if (!solveIKNumerical(target_T, s, q)) {
            continue;
        }

        q = normalizeJoints(q);

        if (!isWithinJointLimits(q)) {
            continue;
        }

        if (!validateSolution(target_T, q)) {
            continue;
        }

        bool dup = false;
        for (const auto& existing : solutions) {
            if (isDuplicate(existing.joints, q, 1e-3)) {
                dup = true;
                break;
            }
        }
        if (dup) {
            continue;
        }

        const ConfigFlags cfg = classifyConfig(q);

        IkSolutionInternal sol;
        sol.joints = q;
        sol.cost = solutionCost(q, seed);
        sol.shoulder = cfg.shoulder;
        sol.elbow = cfg.elbow;
        sol.wrist = cfg.wrist;
        sol.valid = true;

        solutions.push_back(sol);
    }

    std::sort(solutions.begin(), solutions.end(),
              [](const IkSolutionInternal& a, const IkSolutionInternal& b) {
                  return a.cost < b.cost;
              });

    if (static_cast<int>(solutions.size()) > max_solutions) {
        solutions.resize(static_cast<size_t>(max_solutions));
    }

    return !solutions.empty();
}

}  // namespace cs625_kinematics
