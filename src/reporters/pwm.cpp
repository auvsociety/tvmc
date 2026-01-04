/**
 * PWM Reporter - BiBi-Sync Version
 * Converts thrust values to PWM and sends to STM32 via BiBi-Sync UART bridge
 */

#include "pwm.h"
#include "../bibi-bridge/bibi_bridge.h"
#include <cmath>
#include <signal.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <cstring>

using namespace PWMReporter;

static ThrusterConfig config;
static std::map<std::string, _1D::MonotonicInterpolator<float>> interpolaters;
static std::vector<Thruster> thrusters;
static BibiByteTopic* thrust_topic = nullptr;
static BibiByteTopic* pwm_topic = nullptr;
static float* thrust_vector = nullptr;
static int32_t* pwm_vector = nullptr;
static std::atomic<bool> running(false);
static BibiRegistry* g_registry = nullptr;

Thruster::Thruster(_1D::MonotonicInterpolator<float> &interp, float min_thrust, float max_thrust) : interpolater(interp)
{
    this->min_thrust = min_thrust;
    this->max_thrust = max_thrust;
    this->clamp = std::min(std::abs(min_thrust), std::abs(max_thrust));
}

int Thruster::compute_pwm(float thrust)
{
    // clamp = min(max thrust on both sides)
    // thrust = thrust as some unit/spec (usually percentage 0 - 100)
    // full thrust = max of this unit/spec

    // we first find normalized thrust from 0-1, (thrust / full thrust)
    // and the multiply it by the clamp
    // and then restrict this to max or min thrust that can be output by the thruster
    // and then finally interpolate this to PWM 

    return std::round(interpolater(
        std::min(
            std::max(
                clamp * thrust / config.spec.full_thrust,
                this->min_thrust),
            this->max_thrust))) + config.pwm_offset;
}

void PWMReporter::init(BibiRegistry* registry)
{
    g_registry = registry;
    config = loadThrusterConfig();
    
    // create an interpolater for each thrust map
    for (auto map : config.thrust_maps)
    {
        _1D::MonotonicInterpolator<float> interp;
        interp.setData(map.second.thrust, map.second.pwm);
        interpolaters[map.first] = interp;
    }

    // create thrusters
    for (auto tx : config.spec.thruster_types)
    {
        if (interpolaters.find(tx) == interpolaters.end())
        {
            std::cerr << "[PWMReporter] Unable to find thrust map for thruster type " << tx << std::endl;
            exit(1);
        }

        _1D::MonotonicInterpolator<float> &interp = interpolaters[tx];
        float min = *min_element(std::begin(config.thrust_maps[tx].thrust), std::end(config.thrust_maps[tx].thrust));
        float max = *max_element(std::begin(config.thrust_maps[tx].thrust), std::end(config.thrust_maps[tx].thrust));

        thrusters.push_back(Thruster(interp, min, max));
    }

    std::cout << "[PWMReporter] Loaded thruster configuration" << std::endl;

    // Allocate vectors
    thrust_vector = new float[config.spec.number_of_thrusters];
    pwm_vector = new int32_t[config.spec.number_of_thrusters];
    
    for (int i = 0; i < config.spec.number_of_thrusters; i++) {
        thrust_vector[i] = 0;
        pwm_vector[i] = 1500; // neutral PWM
    }

    // Create BiBi-Sync topics
    thrust_topic = bibi_registry_get_byte_topic(registry, bibi::topics::THRUST, 16);
    pwm_topic = bibi_registry_get_byte_topic(registry, bibi::topics::PWM, 16);

    std::cout << "[PWMReporter] Initialized with BiBi-Sync" << std::endl;
    std::cout << "[PWMReporter] Subscribing to: " << bibi::topics::THRUST << std::endl;
    std::cout << "[PWMReporter] Publishing to: " << bibi::topics::PWM << std::endl;
}

void PWMReporter::shutdown()
{
    running = false;
    
    // Zero all thrusters before shutdown
    if (pwm_topic && pwm_vector) {
        for (int i = 0; i < config.spec.number_of_thrusters; i++) {
            pwm_vector[i] = thrusters[i].compute_pwm(0);
        }
        bibi_byte_topic_publish(pwm_topic, reinterpret_cast<uint8_t*>(pwm_vector), 
                                config.spec.number_of_thrusters * sizeof(int32_t));
    }

    if (thrust_topic) {
        bibi_byte_topic_free(thrust_topic);
        thrust_topic = nullptr;
    }
    if (pwm_topic) {
        bibi_byte_topic_free(pwm_topic);
        pwm_topic = nullptr;
    }
    if (thrust_vector) {
        delete[] thrust_vector;
        thrust_vector = nullptr;
    }
    if (pwm_vector) {
        delete[] pwm_vector;
        pwm_vector = nullptr;
    }

    std::cout << "[PWMReporter] Shutdown complete" << std::endl;
}

void PWMReporter::run()
{
    running = true;
    
    while (running)
    {
        // Check for new thrust values
        if (bibi_byte_topic_has_new(thrust_topic)) {
            size_t actual_len;
            uint64_t epoch;
            
            if (bibi_byte_topic_receive(thrust_topic, reinterpret_cast<uint8_t*>(thrust_vector),
                                        config.spec.number_of_thrusters * sizeof(float),
                                        &actual_len, &epoch) == 0) 
            {
                bool change = false;
                static float last_thrust[6] = {0};
                
                // Check if there's any change
                for (int i = 0; i < config.spec.number_of_thrusters && !change; i++) {
                    if (thrust_vector[i] != last_thrust[i]) {
                        change = true;
                    }
                }
                
                if (change) {
                    // Update last thrust
                    std::memcpy(last_thrust, thrust_vector, config.spec.number_of_thrusters * sizeof(float));
                    
                    // Compute PWM values
                    for (int i = 0; i < config.spec.number_of_thrusters; i++) {
                        pwm_vector[i] = thrusters[i].compute_pwm(thrust_vector[i]);
                    }
                    
                    // Publish PWM values
                    bibi_byte_topic_publish(pwm_topic, reinterpret_cast<uint8_t*>(pwm_vector),
                                            config.spec.number_of_thrusters * sizeof(int32_t));
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / PWM_REPORTING_FREQ));
    }
}

// Signal handler for graceful shutdown
static void handle_sigint(int sig)
{
    std::cout << "\n[PWMReporter] Received SIGINT, shutting down..." << std::endl;
    PWMReporter::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    std::cout << "[PWMReporter] BiBi-Sync PWM Reporter starting..." << std::endl;
    
    // Create BiBi-Sync bridge
    bibi::Bridge bridge;
    
    // Initialize
    PWMReporter::init(bridge.get_registry());
    
    // Register signal handler
    signal(SIGINT, handle_sigint);
    
    // Run main loop
    PWMReporter::run();
    
    return 0;
}
