#ifndef SEMANTIC_HPP_
#define SEMANTIC_HPP_

#include "vehicle.hpp"
#include "lane.hpp"

// 语义道路类
struct SemanticLane {
    int id;
    int l_lane_id;
    bool l_change_avbl;
    int r_lane_id;
    bool r_change_avbl;
    double length;
    Lane lane;
};

// 语义道路类合集
struct SemanticLaneSet {
    std::unordered_map<int, SemanticLane> semantic_lanes;

    /**
     * @brief Return the size of container
     *
     * @return int size
     */
    inline int size() const {
        return semantic_lanes.size(); 
    }

    /**
     * @brief Clear the container
     */
    void clear() { 
        semantic_lanes.clear(); 
    }

    /**
     * @brief Print info
     */
    void print() const;
};

/**
 * @brief Vehicle with semantic info
 */
struct SemanticVehicle {
    // * vehicle
    Vehicle vehicle;

    // * nearest lane info
    int nearest_lane_id{kInvalidLaneId};
    double dist_to_lane{-1.0};
    double arc_len_onlane{-1.0};
    double angle_diff_onlane{0.0};
    Lane lane;
};

struct SemanticVehicleSet {
    std::unordered_map<int, SemanticVehicle> semantic_vehicles;
};

#endif  // SEMANTIC_HPP_