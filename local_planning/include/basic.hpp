#ifndef BASIC_HPP_
#define BASIC_HPP_

#include <math.h>
#include <stdio.h>
#include <vector>
#include <array>
#include <map>
#include <unordered_map>
#include <iomanip>
#include <iostream>
#include <memory>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <assert.h>
#include <ros/package.h>
#include <chrono>
#include <filesystem>

enum ErrorType { kSuccess = 0, kWrongStatus, kIllegalInput, kUnknown };

// Eigen aliasing
template <int N>
using Vecf = Eigen::Matrix<double, N, 1>;

template <int N>
using Veci = Eigen::Matrix<int, N, 1>;

template <int M, int N>
using Matf = Eigen::Matrix<double, M, N>;

template <int M, int N>
using Mati8 = Eigen::Matrix<uint8_t, M, N>;

template <int M, int N>
using Mati = Eigen::Matrix<int, M, N>;

template <int N>
using MatNf = Matf<N, N>;

template <int N>
using MatDNf = Eigen::Matrix<double, Eigen::Dynamic, N>;

using MatDf = Matf<Eigen::Dynamic, Eigen::Dynamic>;
using MatDi = Mati<Eigen::Dynamic, Eigen::Dynamic>;
using MatDi8 = Mati8<Eigen::Dynamic, Eigen::Dynamic>;

using Mat2f = Matf<2, 2>;
using Mat3f = Matf<3, 3>;
using Mat4f = Matf<4, 4>;

using Vec2f = Vecf<2>;
using Vec3f = Vecf<3>;
using Vec4f = Vecf<4>;

using Vec2i = Veci<2>;
using Vec3i = Veci<3>;
using Vec4i = Veci<4>;

// Workaround with STL container with eigen with fixed size eigen vector
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template <int N>
using vec_Vecf = vec_E<Vecf<N>>;

const double kBigEPS = 1e-1;

const double kEPS = 1e-6;

const double kSmallEPS = 1e-10;

const double kPi = acos(-1.0);

const double kInf = 1e20;

const int kInvalidAgentId = -1;

const int kInvalidLaneId = -1;

#endif // BASIC_HPP_