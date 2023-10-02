#ifndef CALCULATION_HPP_
#define CALCULATION_HPP_

#include "common.hpp"

inline double truncate(const double& val_in, const double& lower, const double& upper) {
    if (lower > upper) {
        printf("[Calculations]Invalid input!\n");
        throw;
    }
    double res = val_in;
    res = std::max(res, lower);
    res = std::min(res, upper);
    return res;
}

inline double normalize_angle(const double& theta) {
  double theta_tmp = theta;
  theta_tmp -= (theta >= kPi) * 2 * kPi;
  theta_tmp += (theta < -kPi) * 2 * kPi;
  return theta_tmp;
}

inline long long fac(int n) {
    if (n == 0) return 1;
    if (n == 1) return 1;
    if (n == 2) return 2;
    if (n == 3) return 6;
    if (n == 4) return 24;
    if (n == 5) return 120;

    long long ans = 1;
    for (int i = 1; i <= n; i++) {
        ans *= i;
    }
    return ans;
}

inline Vecf<2> rotate_vector_2d(const Vecf<2>& v, const double angle) {
    return Vecf<2>(v[0] * cos(angle) - v[1] * sin(angle), v[0] * sin(angle) + v[1] * cos(angle));
}

inline double vec2d_to_angle(const Vecf<2>& v) { 
    return atan2(v[1], v[0]);
}

template <typename T>
inline void GetRangeVector(const T& lb, const T& ub, const T& step, const bool& if_inc_tail, std::vector<T>* vec) {
    vec->clear();
    int num = std::ceil((ub - lb - kEPS) / step);
    for (int i = 0; i < num; ++i) {
        vec->push_back(lb + i * step);
    }
    if (if_inc_tail) {
        vec->push_back(ub);
    }
}

// 采样函数
inline std::vector<double> linspace(double start_in, double end_in, int num_in) {

    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i){
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end);
    return linspaced;
}

template <class MatrixT>
bool isPsd(const MatrixT& A) {
    if (!A.isApprox(A.transpose())) {
        return false;
    }
    const auto ldlt = A.template selfadjointView<Eigen::Upper>().ldlt();
    if (ldlt.info() == Eigen::NumericalIssue || !ldlt.isPositive()) {
        return false;
    }
    return true;
}

#endif  // CALCULATION_HPP_
