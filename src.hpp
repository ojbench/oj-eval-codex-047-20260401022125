#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include "math.h"
#include <algorithm>
#include <vector>

class Monitor; // forward declaration (provided by OJ before including this file)

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) { pos_cur = _pos_cur; }
    void set_v_cur(const Vec &_v_cur) { v_cur = _v_cur; }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Check if using candidate velocity `v_candidate` would cause a collision
    // within the next TIME_INTERVAL with any other robot, assuming others keep
    // their last observed velocities.
    bool has_predicted_collision(const Vec &v_candidate) const {
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec other_pos = monitor->get_pos_cur(j);
            Vec other_v = monitor->get_v_cur(j);

            Vec delta_pos = pos_cur - other_pos;
            Vec delta_v = v_candidate - other_v;

            double dv_norm = delta_v.norm();
            double project = delta_pos.dot(delta_v);
            if (project >= 0) {
                // Moving apart or stationary relative movement
                continue;
            }
            // Time (projected along approach) to closest point
            double t_proj = (-project) / std::max(dv_norm, 1e-9);

            double min_dis_sqr;
            double delta_r = r + monitor->get_r(j);
            if (t_proj < dv_norm * TIME_INTERVAL) {
                // Closest within interval
                min_dis_sqr = std::max(delta_pos.norm_sqr() - t_proj * t_proj, 0.0);
            } else {
                // Closest at the end of interval
                Vec end_delta = delta_pos + delta_v * TIME_INTERVAL;
                min_dis_sqr = end_delta.norm_sqr();
            }
            if (min_dis_sqr <= delta_r * delta_r - EPSILON) {
                return true;
            }
        }
        return false;
    }

    // Clamp speed magnitude to v_max (minus a tiny epsilon for safety)
    Vec clamp_speed(const Vec &v) const {
        double speed = v.norm();
        double lim = std::max(0.0, v_max - 1e-9);
        if (speed <= lim) return v;
        return v * (lim / speed);
    }

public:
    Vec get_v_next() {
        // If already at target, stop.
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) {
            return Vec(0, 0);
        }

        // Desired straight-line velocity toward target, try to arrive exactly.
        double desired_speed = std::min(v_max, dist / TIME_INTERVAL);
        Vec base_dir = to_tar.normalize();
        Vec direct_v = base_dir * desired_speed;
        direct_v = clamp_speed(direct_v);

        // Priority heuristic: higher IDs yield more aggressively.
        // We consider a set of candidate directions and speeds, choose the first
        // that avoids predicted collisions using last observed states.
        // Angles in radians for detours.
        static const double deg = PI / 180.0;
        std::vector<double> angles = {0.0, 20 * deg, -20 * deg, 40 * deg, -40 * deg, 60 * deg, -60 * deg, 90 * deg, -90 * deg};

        // Speed levels: try direct, then slower.
        std::vector<double> speed_factors;
        // Lower IDs keep more speed, higher IDs yield more.
        // Build a simple sequence decreasing to 0.
        speed_factors.push_back(1.0);
        speed_factors.push_back(0.75);
        speed_factors.push_back(0.5);
        speed_factors.push_back(0.25);
        speed_factors.push_back(0.0);

        // Try direct velocity first if safe.
        if (!has_predicted_collision(direct_v)) {
            return direct_v;
        }

        // Try detours/slowdowns.
        for (double sf : speed_factors) {
            double sp = desired_speed * sf;
            for (double ang : angles) {
                // For zero speed, just test stopping.
                Vec dir_vec = (sf > 0 ? base_dir.rotate(ang).normalize() : Vec(0, 0));
                Vec cand = dir_vec * sp;
                cand = clamp_speed(cand);
                if (!has_predicted_collision(cand)) {
                    return cand;
                }
            }
        }

        // As a last resort, stop.
        return Vec(0, 0);
    }
};

// You may add helper types here if needed.

#endif // PPCA_SRC_HPP

