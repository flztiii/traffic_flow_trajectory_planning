#include "lane.hpp"

// 根据采样点插值生成道路
ErrorType GetLaneBySampleInterpolation(const vec_Vecf<LaneDim>& samples, const std::vector<double>& para, Lane* lane) {
    Spline<LaneDegree, LaneDim> spline;
    SplineGenerator<LaneDegree, LaneDim> spline_generator;

    if (spline_generator.GetCubicSplineBySampleInterpolation(samples, para, &spline) != kSuccess) {
        // printf("[LaneBySampleInterpolation]Cannot get lane by interpolation.\n");
        return kWrongStatus;
    }
    lane->set_position_spline(spline);
    if (!lane->IsValid()) {
        return kWrongStatus;
    }
    return kSuccess;
}

// 根据采样点插值生成道路
ErrorType GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples, Lane* lane) {
    std::vector<double> para;
    double d = 0;
    para.push_back(d);
    for (int i = 1; i < (int)samples.size(); ++i) {
        double dx = samples[i](0) - samples[i - 1](0);
        double dy = samples[i](1) - samples[i - 1](1);
        d += std::hypot(dx, dy);
        para.push_back(d);
    }
    if (GetLaneBySampleInterpolation(samples, para, lane) != kSuccess) {
        // printf("Cannot get lane.\n");
        return kWrongStatus;
    }
    return kSuccess;
}

// 在道路上进行采样
ErrorType SampleLane(const Lane &lane, const double &s0, const double &s1, const double &step, vec_E<Vecf<2>> *samples, double *accum_dist) {
    Vecf<2> pt;
    for (double s = s0; s < s1; s += step) {
        lane.GetPositionByArcLength(s, &pt);
        samples->push_back(pt);
        (*accum_dist) += step;
    }
    return kSuccess;
}

// 计算道路与给定状态的距离
ErrorType GetDistanceToLanesUsing3DofState(const Vec3f &state, const Lane &lane,std::tuple<double, double, double> &res) {
    double arc_len;
    lane.GetArcLengthByVecPosition(Vec2f(state(0), state(1)), &arc_len);

    double lane_angle;
    lane.GetOrientationByArcLength(arc_len, &lane_angle);
    double angle_diff = normalize_angle(state(2) - lane_angle);
    
    Vec2f pt;
    lane.GetPositionByArcLength(arc_len, &pt);
    double dist;
    if (-(state(0) - pt(0)) * sin(lane_angle) + (state(1) - pt(1)) * cos(lane_angle) > 0) {
        dist = std::hypot((state(0) - pt(0)), (state(1) - pt(1)));
    } else {
        dist = -std::hypot((state(0) - pt(0)), (state(1) - pt(1)));
    }

    res = std::tuple<double, double, double>(dist, arc_len, angle_diff);
    return kSuccess;
}

// 可视化道路
visualization_msgs::Marker visualizeLane(const Lane &lane, int id) {
    // 得到路径的长度
    double lane_length = lane.end() - lane.begin();
    // 计算采样点数量
    int sample_num = std::min(1000, std::max(2, static_cast<int>(lane_length / 1.0)));
    // 得到采样结果
    vec_E<Vecf<2>> samples;
    double accum_dist;
    SampleLane(lane, lane.begin(), lane.end(), lane_length / static_cast<double>(sample_num), &samples, &accum_dist);
    // 格式化
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.id = id;
    marker.scale.x = 0.05;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1;
    for (auto sample: samples) {
        geometry_msgs::Point point;
        point.x = sample(0);
        point.y = sample(1);
        marker.points.push_back(point);
    }
    return marker;
}
