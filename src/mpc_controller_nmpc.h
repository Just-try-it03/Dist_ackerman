#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include "jarvis_msgs/RefPoint.h"
class SimpleFormationNMPCController {
public:
//    struct RefPoint {
//        double x;
//        double y;
//        double yaw;
//        double v;   // 期望速度，可选
//    };
    typedef jarvis_msgs::RefPoint RefPoint;
    struct LeaderPoint {
        double x;
        double y;
        double yaw;
        double v;      // 可选
        double omega;  // 可选
    };

    struct FormationConfig {
        double dx_des;
        double dy_des;
        double dyaw_des;

        double q_pos;  // 位置误差权重
        double q_yaw;  // 航向误差权重

        FormationConfig()
            : dx_des(0.0), dy_des(0.0), dyaw_des(0.0),
              q_pos(0.0), q_yaw(0.0) {}
    };

    SimpleFormationNMPCController()
        : horizon_(10),
          dt_(0.1),
          q_pos_(5.0),
          q_yaw_(2.0),
          q_terminal_(10.0),
          r_v_(0.1),
          r_omega_(0.1),
          max_v_(0.8),
          max_omega_(1.5),
          max_iterations_(20),
          step_size_(0.1),
          finite_diff_eps_(1e-2),
          formation_enabled_(false)
    {}

    /// 设置 MPC 基本参数
    void setParameters(int horizon, double dt,
                       double q_pos, double q_yaw, double q_terminal,
                       double r_v, double r_omega)
    {
        horizon_      = std::max(1, horizon);
        dt_           = dt;
        q_pos_        = q_pos;
        q_yaw_        = q_yaw;
        q_terminal_   = q_terminal;
        r_v_          = r_v;
        r_omega_      = r_omega;
    }

    /// 设置速度约束
    void setLimits(double max_v, double max_omega)
    {
        max_v_     = max_v;
        max_omega_ = max_omega;
    }

    /// 设置优化参数
    void setSolverOptions(int max_iterations,
                          double step_size,
                          double finite_diff_eps)
    {
        max_iterations_  = max_iterations;
        step_size_       = step_size;
        finite_diff_eps_ = finite_diff_eps;
    }

    /// 设置路径（master 和 slave 都可以用）
    void setReferencePath(const std::vector<RefPoint>& path)
    {
        ref_path_ = path;
    }

    /// 设置编队配置（只在 slave 上启用）
    void setFormationConfig(const FormationConfig& cfg)
    {
        formation_cfg_   = cfg;
        formation_enabled_ = (cfg.q_pos > 0.0 || cfg.q_yaw > 0.0);
    }

    void setLeaderTrajectory(const std::vector<LeaderPoint>& traj)
    {
        leader_traj_ = traj;
    }

    /// 主入口：给定当前位姿 [x, y, yaw]，输出 [v, omega]
    std::vector<double> computeControl(const std::vector<double>& current_pose)
    {
        std::vector<double> cmd(2, 0.0);

        if (ref_path_.size() < 2) {
            // 没有路径，直接停下
            return cmd;
        }

        // 1. 找到当前路径上最近点
        int nearest_idx = findClosestIndex(current_pose[0], current_pose[1]);
        if (nearest_idx < 0) {
            return cmd;
        }

        // 2. 构造预测时域上的参考路径（N+1）
        buildHorizonRef(nearest_idx);

        // 3. 构造预测时域上的 leader 轨迹（N+1），仅在编队模式下使用
        buildLeaderHorizon();

        // 4. 当前状态
        MPCState x0;
        x0.x   = current_pose[0];
        x0.y   = current_pose[1];
        x0.yaw = current_pose[2];

        // 5. 求解 NMPC
        std::vector<MPCControl> u_seq;
        if (!solveNMPC(x0, u_seq)) {
            return cmd;
        }

        // 6. 只取第一步控制量
        cmd[0] = clamp(u_seq[0].v, -max_v_,     max_v_);
        cmd[1] = clamp(u_seq[0].omega, -max_omega_, max_omega_);

        last_u_seq_ = u_seq;
        return cmd;
    }
    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

private:
    struct MPCState {
        double x;
        double y;
        double yaw;
    };

    struct MPCControl {
        double v;
        double omega;
    };

    // ==== MPC 参数 ====
    int    horizon_;
    double dt_;
    double q_pos_;
    double q_yaw_;
    double q_terminal_;
    double r_v_;
    double r_omega_;
    double max_v_;
    double max_omega_;

    int    max_iterations_;
    double step_size_;
    double finite_diff_eps_;

    // ==== 路径 & 编队数据 ====
    std::vector<RefPoint>   ref_path_;       // 全局/局部路径
    std::vector<RefPoint>   horizon_ref_;    // 当前预测时域路径（N+1）

    std::vector<LeaderPoint> leader_traj_;   // leader 提供的未来轨迹（任意长度）
    std::vector<LeaderPoint> leader_horizon_; // 当前预测时域 leader 轨迹（N+1）

    FormationConfig         formation_cfg_;
    bool                    formation_enabled_;

    std::vector<MPCControl> last_u_seq_;     // warm start


    // ==== 工具函数 ====
//    static double normalizeAngle(double angle)
//    {
//        while (angle > M_PI)  angle -= 2.0 * M_PI;
//        while (angle < -M_PI) angle += 2.0 * M_PI;
//        return angle;
//    }

    static double clamp(double v, double vmin, double vmax)
    {
        return std::max(vmin, std::min(v, vmax));
    }

    int findClosestIndex(double x, double y) const
    {
        if (ref_path_.empty()) return -1;

        double min_dist = std::numeric_limits<double>::max();
        int    min_idx  = -1;
        for (size_t i = 0; i < ref_path_.size(); ++i) {
            double dx = x - ref_path_[i].x;
            double dy = y - ref_path_[i].y;
            double d2 = dx*dx + dy*dy;
            if (d2 < min_dist) {
                min_dist = d2;
                min_idx  = static_cast<int>(i);
            }
        }
        return min_idx;
    }

    void buildHorizonRef(int start_idx)
    {
        horizon_ref_.clear();
        horizon_ref_.reserve(horizon_ + 1);

        for (int k = 0; k <= horizon_; ++k) {
            int idx = start_idx + k;
            if (idx >= static_cast<int>(ref_path_.size())) {
                idx = static_cast<int>(ref_path_.size()) - 1;
            }
            horizon_ref_.push_back(ref_path_[idx]);
        }
    }

    void buildLeaderHorizon()
    {
        leader_horizon_.clear();
        leader_horizon_.reserve(horizon_ + 1);

        if (!formation_enabled_) {
            // master 情况：不需要 leader 轨迹
            return;
        }

        if (leader_traj_.empty()) {
            // 没有 leader 参考，编队 cost 只能暂时忽略
            return;
        }

        // 假设 leader_traj_ 本身就是从当前时刻往后预测的序列
        // 这里简单取前 N+1 个，不足则用最后一个补齐
        for (int k = 0; k <= horizon_; ++k) {
            int idx = k;
            if (idx >= static_cast<int>(leader_traj_.size())) {
                idx = static_cast<int>(leader_traj_.size()) - 1;
            }
            leader_horizon_.push_back(leader_traj_[idx]);
        }
    }

    MPCState simulateStep(const MPCState& x, const MPCControl& u) const
    {
        MPCState x_next;
        x_next.x   = x.x + u.v * std::cos(x.yaw) * dt_;
        x_next.y   = x.y + u.v * std::sin(x.yaw) * dt_;
        x_next.yaw = normalizeAngle(x.yaw + u.omega * dt_);
        return x_next;
    }

    // 计算 cost：包含路径误差 + 控制量 + 终端 +（可选）编队误差
    double computeCost(const MPCState& x0,
                       const std::vector<MPCControl>& u_seq)
    {
        MPCState x = x0;
        double J = 0.0;

        const int N = horizon_;

        for (int k = 0; k < N; ++k) {
            const RefPoint& r = horizon_ref_[k];

            double ex   = x.x   - r.x;
            double ey   = x.y   - r.y;
            double eyaw = normalizeAngle(x.yaw - r.yaw);

            // 路径跟踪 cost
            J += q_pos_ * (ex*ex + ey*ey)
               + q_yaw_ * (eyaw*eyaw)
               + r_v_   * (u_seq[k].v    * u_seq[k].v)
               + r_omega_ * (u_seq[k].omega * u_seq[k].omega);

            // 编队 cost：相对 leader 的误差
            if (formation_enabled_ && (k < static_cast<int>(leader_horizon_.size()))) {
                const LeaderPoint& L = leader_horizon_[k];

                // 从世界坐标系转到 leader 车体坐标系
                double dx = x.x - L.x;
                double dy = x.y - L.y;
                double cosL = std::cos(L.yaw);
                double sinL = std::sin(L.yaw);

                double dx_rel =  cosL * dx + sinL * dy;
                double dy_rel = -sinL * dx + cosL * dy;

                double ex_form = dx_rel - formation_cfg_.dx_des;
                double ey_form = dy_rel - formation_cfg_.dy_des;
                double eyaw_form = normalizeAngle(
                    x.yaw - L.yaw - formation_cfg_.dyaw_des
                );

                J += formation_cfg_.q_pos * (ex_form*ex_form + ey_form*ey_form)
                   + formation_cfg_.q_yaw * (eyaw_form*eyaw_form);
            }

            x = simulateStep(x, u_seq[k]);
        }

        // 终端 cost（路径）
        const RefPoint& rN = horizon_ref_[N];
        double exN   = x.x   - rN.x;
        double eyN   = x.y   - rN.y;
        double eyawN = normalizeAngle(x.yaw - rN.yaw);
        J += q_terminal_ * (exN*exN + eyN*eyN + eyawN*eyawN);

        // 终端编队 cost
        if (formation_enabled_ && (N < static_cast<int>(leader_horizon_.size()))) {
            const LeaderPoint& LN = leader_horizon_[N];
            double dx = x.x - LN.x;
            double dy = x.y - LN.y;
            double cosL = std::cos(LN.yaw);
            double sinL = std::sin(LN.yaw);

            double dx_rel =  cosL * dx + sinL * dy;
            double dy_rel = -sinL * dx + cosL * dy;

            double ex_form = dx_rel - formation_cfg_.dx_des;
            double ey_form = dy_rel - formation_cfg_.dy_des;
            double eyaw_form = normalizeAngle(
                x.yaw - LN.yaw - formation_cfg_.dyaw_des
            );

            J += formation_cfg_.q_pos * (ex_form*ex_form + ey_form*ey_form)
               + formation_cfg_.q_yaw * (eyaw_form*eyaw_form);
        }

        return J;
    }

    bool solveNMPC(const MPCState& x0,
                   std::vector<MPCControl>& u_seq)
    {
        const int N = horizon_;

        // 初始控制序列（用上一周期 warm start）
        if (last_u_seq_.size() == static_cast<size_t>(N)) {
            u_seq = last_u_seq_;
        } else {
            u_seq.assign(N, MPCControl{0.0, 0.0});
        }

        std::vector<MPCControl> grad(N);

        for (int iter = 0; iter < max_iterations_; ++iter) {
            double base_cost = computeCost(x0, u_seq);

            // 有限差分计算梯度
            for (int k = 0; k < N; ++k) {
                // v 方向
                {
                    std::vector<MPCControl> u_plus  = u_seq;
                    std::vector<MPCControl> u_minus = u_seq;

                    u_plus[k].v  += finite_diff_eps_;
                    u_minus[k].v -= finite_diff_eps_;

                    u_plus[k].v  = clamp(u_plus[k].v,  -max_v_, max_v_);
                    u_minus[k].v = clamp(u_minus[k].v, -max_v_, max_v_);

                    double J_plus  = computeCost(x0, u_plus);
                    double J_minus = computeCost(x0, u_minus);

                    grad[k].v = (J_plus - J_minus) / (2.0 * finite_diff_eps_);
                }

                // omega 方向
                {
                    std::vector<MPCControl> u_plus  = u_seq;
                    std::vector<MPCControl> u_minus = u_seq;

                    u_plus[k].omega  += finite_diff_eps_;
                    u_minus[k].omega -= finite_diff_eps_;

                    u_plus[k].omega  = clamp(u_plus[k].omega,  -max_omega_, max_omega_);
                    u_minus[k].omega = clamp(u_minus[k].omega, -max_omega_, max_omega_);

                    double J_plus  = computeCost(x0, u_plus);
                    double J_minus = computeCost(x0, u_minus);

                    grad[k].omega = (J_plus - J_minus) / (2.0 * finite_diff_eps_);
                }
            }

            // 梯度下降 + 投影
            for (int k = 0; k < N; ++k) {
                u_seq[k].v     -= step_size_ * grad[k].v;
                u_seq[k].omega -= step_size_ * grad[k].omega;

                u_seq[k].v     = clamp(u_seq[k].v, -max_v_,     max_v_);
                u_seq[k].omega = clamp(u_seq[k].omega, -max_omega_, max_omega_);
            }

            (void)base_cost; // 你可以打印 base_cost 来观察收敛
        }

        return true;
    }
};
