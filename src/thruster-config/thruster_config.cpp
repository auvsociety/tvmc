#include "thruster_config.h"
#include <fstream>
#include "json.hpp"
#include "csv.h"
#include <cstdlib>
#include <iostream>
#include <filesystem>

using json = nlohmann::json;

// Get the config directory path
static std::string getConfigPath() {
    // First check environment variable
    const char* config_dir = std::getenv("TVMC_CONFIG_DIR");
    if (config_dir) {
        return std::string(config_dir);
    }
    
    // Try relative path from executable
    std::filesystem::path exe_path = std::filesystem::current_path();
    
    // Check various possible locations
    std::vector<std::string> paths = {
        "./config",
        "../config",
        "../../config",
        "/ros-ws/src/tvmc/config",
        exe_path.string() + "/config"
    };
    
    for (const auto& p : paths) {
        if (std::filesystem::exists(p + "/config.json")) {
            return p;
        }
    }
    
    std::cerr << "[ThrusterConfig] ERROR: Could not find config directory" << std::endl;
    std::cerr << "[ThrusterConfig] Set TVMC_CONFIG_DIR environment variable" << std::endl;
    exit(1);
}

ThrusterConfig loadThrusterConfig()
{
    std::string config_dir = getConfigPath();
    std::string path = config_dir + "/config.json";
    std::ifstream f(path);
    
    if (!f.is_open()) {
        std::cerr << "[ThrusterConfig] ERROR: Cannot open config file: " << path << std::endl;
        exit(1);
    }

    std::cout << "[ThrusterConfig] Loading config from: " << path << std::endl;

    ThrusterConfig config;

    json file = json::parse(f);

    if (!file.contains("thrusterSpec"))
    {
        std::cerr << "[ThrusterConfig] ERROR: Unable to find thruster spec." << std::endl;
        exit(1);
    }

    if (!file.contains("thrustVectors"))
    {
        std::cerr << "[ThrusterConfig] ERROR: Unable to find thrust vectors." << std::endl;
        exit(1);
    }

    if (!file.contains("pwmThrustMaps"))
    {
        std::cerr << "[ThrusterConfig] ERROR: Unable to find thrust maps." << std::endl;
        exit(1);
    }

    auto spec = file.at("thrusterSpec");
    auto vectors = file.at("thrustVectors");
    auto thrust_maps = file.at("pwmThrustMaps");

    // read params
    config.spec.number_of_thrusters = spec.at("noOfThrusters");
    config.spec.min_thrust = spec.at("minThrust");
    config.spec.max_thrust = spec.at("maxThrust");
    config.spec.full_thrust = spec.at("fullThrust");
    config.pwm_offset = file.at("pwmOffset");

    // read thruster types
    for (auto &type: spec.at("thrustMaps").items())
    config.spec.thruster_types.push_back(type.value().get<std::string>());

    // read thrustered vectors
    config.vectors.surge = vectors.at("surge").get<std::vector<float>>();
    config.vectors.pitch = vectors.at("pitch").get<std::vector<float>>();
    config.vectors.roll = vectors.at("roll").get<std::vector<float>>();
    config.vectors.yaw = vectors.at("yaw").get<std::vector<float>>();
    config.vectors.heave = vectors.at("heave").get<std::vector<float>>();
    config.vectors.sway = vectors.at("sway").get<std::vector<float>>();

    // read thrust maps
    for (auto &map : thrust_maps.items())
    {
        PWMThrustMap m;

        std::string tmpath = config_dir + "/" + map.value().get<std::string>();
        io::CSVReader<2> csv(tmpath);
        int pwm;
        float thrust;

        while (csv.read_row(pwm, thrust))
        {
            m.thrust.push_back(thrust);
            m.pwm.push_back(pwm);
        }

        config.thrust_maps[map.key()] = m;
    }

    std::cout << "[ThrusterConfig] Loaded " << config.spec.number_of_thrusters 
              << " thrusters" << std::endl;

    return config;
}
