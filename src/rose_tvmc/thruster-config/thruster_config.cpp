#include "thruster_config.h"
#include <fstream>
#include "json.hpp"
#include "csv.h"
#include <ros/package.h>
#include <ros/console.h>

using json = nlohmann::json;

ThrusterConfig loadThrusterConfig()
{
    std::string path = ros::package::getPath("rose_tvmc") + "/config/config.json";
    std::ifstream f(path);

    ThrusterConfig config;

    json file = json::parse(f);

    if (!file.contains("thrusterSpec"))
    {
        ROS_ERROR("Unable to find thruster spec.");
        exit(1);
    }

    if (!file.contains("thrustVectors"))
    {
        ROS_ERROR("Unable to find thrust vectors.");
        exit(1);
    }

    auto spec = file.at("thrusterSpec");
    auto vectors = file.at("thrustVectors");

    // read params 
    config.spec.number_of_thrusters = spec.at("noOfThrusters");
    config.spec.min_pwm = spec.at("minPWM");
    config.spec.max_pwm = spec.at("maxPWM");
    config.spec.zero_thrust_pwm = spec.at("zeroThrustPWM");
    config.spec.min_thrust = spec.at("minThrust");
    config.spec.max_thrust = spec.at("maxThrust");
    config.spec.full_thrust = spec.at("fullThrust");

    config.vectors.surge = vectors.at("surge").get<std::vector<float>>();
    config.vectors.pitch = vectors.at("pitch").get<std::vector<float>>();
    config.vectors.roll = vectors.at("roll").get<std::vector<float>>();
    config.vectors.yaw = vectors.at("yaw").get<std::vector<float>>();
    config.vectors.heave = vectors.at("heave").get<std::vector<float>>();
    config.vectors.sway = vectors.at("sway").get<std::vector<float>>();

    // read thrust map
    std::string thrustMapPath = ros::package::getPath("rose_tvmc") + "/config/" + spec.at("pwmToThrustMap").get<std::string>();
    io::CSVReader<2> csv(thrustMapPath);

    int pwm; float thrust;
    std::vector<float> thrustV;
    std::vector<int> pwmV;

    while (csv.read_row(pwm, thrust)) {
        thrustV.push_back(thrust);
        pwmV.push_back(pwm);
    }

    config.spec.thrust_map_thrust = thrustV;
    config.spec.thrust_map_pwm = pwmV;

    return config;
}
