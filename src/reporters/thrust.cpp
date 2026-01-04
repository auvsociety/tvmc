/**
 * Thrust Reporter - BiBi-Sync Version
 * Publishes thrust values via BiBi-Sync topics
 */

#include "thrust.h"
#include "../thruster-config/thruster_config.h"
#include "../bibi-bridge/bibi_bridge.h"
#include <thread>
#include <atomic>
#include <vector>
#include <iostream>
#include <cstring>

static BibiByteTopic* thrust_topic = nullptr;
static ThrusterConfig config;
static float* thrust_vector = nullptr;
static std::thread reporter_thread;
static std::atomic<bool> running(false);

void ThrustReporterThread()
{
    while (running)
    {
        ThrustReporter::refresh();
        std::this_thread::sleep_for(std::chrono::microseconds(THRUST_REPORT_RATE_US));
    }
}

void ThrustReporter::init(BibiRegistry* registry)
{
    // load thruster config
    config = loadThrusterConfig();

    // ensure thrust_vector is non empty
    thrust_vector = new float[config.spec.number_of_thrusters];
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        thrust_vector[i] = 0;

    // create BiBi-Sync topic for thrust values
    thrust_topic = bibi_registry_get_byte_topic(registry, bibi::topics::THRUST, 16);

    std::cout << "[ThrustReporter] Initialized with BiBi-Sync, publishing to " 
              << bibi::topics::THRUST << std::endl;

    // start reporter thread
    running = true;
    reporter_thread = std::thread(ThrustReporterThread);
}

void ThrustReporter::refresh()
{
    if (!thrust_topic) return;
    
    // Publish thrust values as a packed float array
    bibi_byte_topic_publish(thrust_topic, 
                            reinterpret_cast<uint8_t*>(thrust_vector),
                            config.spec.number_of_thrusters * sizeof(float));
}

void ThrustReporter::report(float *tvec)
{
    // copy the vector onto local variable
    std::memcpy(thrust_vector, tvec, config.spec.number_of_thrusters * sizeof(float));
}

void ThrustReporter::kill()
{
    // stop thread
    running = false;
    if (reporter_thread.joinable()) {
        reporter_thread.join();
    }

    // free topic
    if (thrust_topic) {
        bibi_byte_topic_free(thrust_topic);
        thrust_topic = nullptr;
    }

    // free allocated vector
    if (thrust_vector) {
        delete[] thrust_vector;
        thrust_vector = nullptr;
    }
    
    std::cout << "[ThrustReporter] Shutdown complete" << std::endl;
}

// backwards-compatibility

void ThrustReporter::writeThrusterValues(float *thrust_vector)
{
    return ThrustReporter::report(thrust_vector);
}

void ThrustReporter::shutdown()
{
    return ThrustReporter::kill();
}
