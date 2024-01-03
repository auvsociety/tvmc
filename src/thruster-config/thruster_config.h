#ifndef THRUSTER_CONFIG_H
#define THRUSTER_CONFIG_H

#include <vector>
#include <chrono>

typedef struct ThrusterSpec
{
    int number_of_thrusters;
    int max_thrust;
    int min_thrust;
    int full_thrust;
    int zero_thrust_pwm;
    int min_pwm;
    int max_pwm;
    std::vector<float> thrust_map_thrust;
    std::vector<int> thrust_map_pwm;
} ThrusterSpec;

typedef struct ThrustVectors
{
    std::vector<float> surge;
    std::vector<float> heave;
    std::vector<float> sway;
    std::vector<float> roll;
    std::vector<float> pitch;
    std::vector<float> yaw;

} ThrustVectors;

typedef struct ThrusterConfig
{
    ThrusterSpec spec;
    ThrustVectors vectors;

} ThrusterConfig;

ThrusterConfig loadThrusterConfig();

#endif