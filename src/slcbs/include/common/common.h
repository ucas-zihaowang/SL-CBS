
#ifndef COMMON_H_
#define COMMON_H_

#include "data_struct.h"
#include "math.h"
#include "rootfinder.h"
#include "timer.h"

enum ControlMode
{
    NONE, VEL, ACC, JRK, SNP
};

static std::vector<std::string> cmtostr = { "NONE", "VEL", "ACC", "JRK", "SNP" };
static std::unordered_map<std::string, ControlMode> strtocm = {
    {"NONE", ControlMode::NONE}, {"VEL", ControlMode::VEL}, {"ACC", ControlMode::ACC},
    {"JRK", ControlMode::JRK}, {"SNP", ControlMode::SNP}
};

#endif
