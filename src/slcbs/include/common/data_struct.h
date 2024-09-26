
#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H


#define ANSI_COLOR_RED      "\x1b[1;31m" 
#define ANSI_COLOR_GREEN    "\x1b[1;32m" 
#define ANSI_COLOR_YELLOW   "\x1b[1;33m" 
#define ANSI_COLOR_BLUE     "\x1b[1;34m"
#define ANSI_COLOR_MAGENTA  "\x1b[1;35m"
#define ANSI_COLOR_CYAN     "\x1b[1;36m"
#define ANSI_COLOR_RESET    "\x1b[0m" 


#include <iostream>
#include <memory>
#include <string>
#include <array>
#include <vector>
#include <list>
#include <map>
#include <utility> 
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <cfloat>
#include <climits>
#define _USE_MATH_DEFINES

#include <boost/functional/hash.hpp>
#include <boost/unordered_set.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/unordered_map.hpp>

#include "nanoflann.hpp"
using namespace nanoflann;
#include "KDTreeVectorOfVectorsAdaptor.h"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <unsupported/Eigen/Polynomials>


template<int N>
using Vecf = Eigen::Matrix<double, N, 1>;
typedef Vecf<2> Vec2f;
typedef Vecf<3> Vec3f;
typedef Vecf<4> Vec4f;
typedef Vecf<6> Vec6f;
typedef Vecf<11> Vec11f;
template<int N>
using Veci = Eigen::Matrix<int, N, 1>;
typedef Veci<2> Vec2i;
typedef Veci<3> Vec3i;
typedef Veci<Eigen::Dynamic> VecDi;
typedef Vecf<Eigen::Dynamic> VecDf;

template<int M, int N>
using Matf = Eigen::Matrix<double, M, N>;
typedef Matf<2, 2> Mat2f;
typedef Matf<3, 3> Mat3f;
typedef Matf<4, 4> Mat4f;
typedef Matf<6, 6> Mat6f;

template<typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
template<int N>
using vec_Vecf = vec_E<Vecf<N>>;
template<int N>
using vec_Veci = vec_E<Veci<N>>;
typedef vec_E<Vec2f> vec_Vec2f;
typedef vec_E<Vec3f> vec_Vec3f;
typedef vec_E<Vec2i> vec_Vec2i;
typedef vec_E<Vec3i> vec_Vec3i;

typedef Eigen::Transform<double, 2, Eigen::Affine> Aff2f;
typedef Eigen::Transform<double, 3, Eigen::Affine> Aff3f;

#endif