
#ifndef COPTER_COMMON_H_
#define COPTER_COMMON_H_

#include "command.h"
#include "copterstate.h"
#include "copterparameter.h"
#include "primitive.h"
#include "solution.h"
#include "trajectory.h"


#define STOP_MOTION_PRIMITVE 0


#define MAX_LOW_ITERATION_NUM 60000


#define CHECK_STATIC_OBS_METHOD 3


// #define SLPLANNER_DEBUG

// #define SLPLANNER_ITER_DEBUG


// #define CBSSEARCH_DEBUG


// #define CBSSEARCH_ITER_DEBUG

#define CBSSEARCH_END_DEBUG


#define MULTI_THREAD

#define STOP_GROUP 20


#define MAX_HIGH_ITERATION_NUM 60000
   
#define CBS_RUNTIME 90000

#define PRIMITIVE_COLLISION_CHECK 1

#endif