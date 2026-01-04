/**
 * BiBi-Sync Bridge for TVMC
 * Replaces ROS communication with BiBi-Sync topics
 */

#ifndef BIBI_BRIDGE_H
#define BIBI_BRIDGE_H

#include <cstdint>
#include <cstring>
#include <functional>

// Include the BiBi-Sync FFI header
extern "C" {
#include "bibi_sync.h"
}

namespace bibi {

// Topic names used by TVMC
namespace topics {
    constexpr const char* COMMAND = "/tvmc/command";
    constexpr const char* CONTROL_MODE = "/tvmc/control_mode";
    constexpr const char* CURRENT_POINT = "/tvmc/current_point";
    constexpr const char* TARGET_POINT = "/tvmc/target_point";
    constexpr const char* PID_CONSTANTS = "/tvmc/pid_constants";
    constexpr const char* PID_LIMITS = "/tvmc/pid_limits";
    constexpr const char* MULTI_THRUST = "/tvmc/multi_thrust";
    constexpr const char* THRUST = "/tvmc/thrust";
    constexpr const char* PWM = "/tvmc/pwm";
    // STM32 sensor topics
    constexpr const char* IMU = "/stm32/imu";
    constexpr const char* ORIENTATION = "/stm32/orientation";
    constexpr const char* DEPTH = "/stm32/depth";
}

// Message structures (must match Python side)
#pragma pack(push, 1)

struct CommandMsg {
    uint8_t command;
};

struct ControlModeMsg {
    uint8_t dof;
    uint8_t mode;
};

struct CurrentPointMsg {
    uint8_t dof;
    float current;
};

struct TargetPointMsg {
    uint8_t dof;
    float target;
};

struct PidConstantsMsg {
    uint8_t dof;
    float kp;
    float ki;
    float kd;
    float acceptable_error;
    float ko;
};

struct PidLimitsMsg {
    uint8_t dof;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
};

struct MultiThrustMsg {
    float surge;
    float sway;
    float heave;
    float roll;
    float pitch;
    float yaw;
};

struct ThrustMsg {
    float thrust[6];  // per-thruster thrust values
};

struct PwmMsg {
    int32_t pwm[6];   // per-thruster PWM values
};

struct ImuMsg {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
};

struct OrientationMsg {
    float roll;
    float pitch;
    float yaw;
};

struct DepthMsg {
    float depth;
};

#pragma pack(pop)

/**
 * BiBi-Sync Topic Wrapper
 */
class Topic {
private:
    BibiByteTopic* topic;
    uint64_t last_epoch;
    
public:
    Topic(BibiRegistry* registry, const char* name, size_t capacity = 32) {
        topic = bibi_registry_get_byte_topic(registry, name, capacity);
        last_epoch = 0;
    }
    
    ~Topic() {
        if (topic) {
            bibi_byte_topic_free(topic);
        }
    }
    
    void publish(const void* data, size_t len) {
        if (topic) {
            bibi_byte_topic_publish(topic, static_cast<const uint8_t*>(data), len);
        }
    }
    
    template<typename T>
    void publish(const T& msg) {
        publish(&msg, sizeof(T));
    }
    
    // Try to receive a message (uses try_receive FFI)
    bool receive(void* data, size_t max_len, size_t* actual_len) {
        if (!topic) return false;
        uintptr_t out_len = 0;
        int32_t result = bibi_byte_topic_try_receive(topic, static_cast<uint8_t*>(data), 
                                                      &out_len, max_len);
        if (result == 0 && actual_len) {
            *actual_len = static_cast<size_t>(out_len);
        }
        return result == 0;
    }
    
    template<typename T>
    bool receive(T* msg) {
        size_t actual_len;
        return receive(msg, sizeof(T), &actual_len) && actual_len >= sizeof(T);
    }
    
    // Peek at latest message without consuming
    bool peek_latest(void* data, size_t max_len, size_t* actual_len, uint64_t* epoch) {
        if (!topic) return false;
        uintptr_t out_len = 0;
        uint64_t out_epoch = 0;
        int32_t result = bibi_byte_topic_peek_latest(topic, static_cast<uint8_t*>(data),
                                                      &out_len, &out_epoch, max_len);
        if (result == 0) {
            if (actual_len) *actual_len = static_cast<size_t>(out_len);
            if (epoch) *epoch = out_epoch;
        }
        return result == 0;
    }
    
    template<typename T>
    bool peek_latest(T* msg, uint64_t* epoch = nullptr) {
        size_t actual_len;
        return peek_latest(msg, sizeof(T), &actual_len, epoch) && actual_len >= sizeof(T);
    }
    
    // Check if there's new data since last receive
    bool has_new() {
        if (!topic) return false;
        uint64_t current_epoch = bibi_byte_topic_latest_epoch(topic);
        return current_epoch > last_epoch;
    }
    
    // Update last seen epoch after processing
    void mark_seen() {
        if (topic) {
            last_epoch = bibi_byte_topic_latest_epoch(topic);
        }
    }
    
    size_t len() {
        if (!topic) return 0;
        return bibi_byte_topic_len(topic);
    }
    
    bool is_empty() {
        if (!topic) return true;
        return bibi_byte_topic_is_empty(topic);
    }
};

/**
 * BiBi-Sync Bridge
 * Central registry and topic management
 */
class Bridge {
private:
    BibiRegistry* registry;
    
public:
    Bridge() {
        registry = bibi_registry_new();
    }
    
    ~Bridge() {
        if (registry) {
            bibi_registry_free(registry);
        }
    }
    
    BibiRegistry* get_registry() { return registry; }
    
    Topic create_topic(const char* name, size_t capacity = 32) {
        return Topic(registry, name, capacity);
    }
};

} // namespace bibi

#endif // BIBI_BRIDGE_H
