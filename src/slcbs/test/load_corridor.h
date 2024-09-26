
#ifndef LOAD_CORRIDOR_H_
#define LOAD_CORRIDOR_H_

#include "common.h"
#include <yaml-cpp/yaml.h>


template <class Ti, class Tf>
class InstanceLoader
{
public:

    explicit InstanceLoader(const std::string &file )
    {
        try
        {
            YAML::Node config = YAML::LoadFile(file);

            const std::vector<double> &start = config[0]["start"].as<std::vector<double>>();
            for (unsigned int i = 0; i < start.size(); i++) start_(i) = start[i];

            const std::vector<double> &goal = config[1]["goal"].as<std::vector<double>>();
            for (unsigned int i = 0; i < goal.size(); i++) goal_(i) = goal[i];

            const std::vector<double> &origin_vec = config[2]["origin"].as<std::vector<double>>();
            for (unsigned int i = 0; i < origin_vec.size(); i++)
                origin_(i) = origin_vec[i];

            const std::vector<int> &dim_vec = config[3]["dim"].as<std::vector<int>>();
            for (unsigned int i = 0; i < dim_vec.size(); i++) dim_(i) = dim_vec[i];

            resolution_ = config[4]["resolution"].as<double>();

            const std::vector<int> &data = config[5]["data"].as<std::vector<int>>();
            data_.resize(data.size());
            for (unsigned int i = 0; i < data.size(); i++)
                data_[i] = data[i] > 0 ? 100 : 0;

            exist_ = true;
        }
        catch (YAML::ParserException &e)
        {
            exist_ = false;
        }
    }

    bool exist() { return exist_; }
    Tf origin() { return origin_; }
    Ti dim() { return dim_; }
    double resolution() { return resolution_; }
    std::vector<signed char> data() { return data_; }
    double start(int i) { return start_(i); }
    double goal(int i) {return goal_(i); }

    void printInstanceInfo()
    {
        std::cout << ANSI_COLOR_YELLOW "********************Instance Info:********************" ANSI_COLOR_RESET << std::endl;
        std::cout << "Start: (" << start_.transpose() << ")" << std::endl;
        std::cout << "Goal: (" << goal_.transpose() << ")" << std::endl;
        std::cout << "Origin: (" << origin_.transpose() << ")" << std::endl;
        std::cout << "Dim: (" << dim_.transpose() << ")" << std::endl;
        std::cout << "Resolution: " << resolution_ << std::endl;
        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }

private:

    Tf start_;
    Tf goal_;
    Tf origin_;
    Ti dim_;
    double resolution_;
    std::vector<signed char> data_;
    bool exist_ = false;
};

#endif
