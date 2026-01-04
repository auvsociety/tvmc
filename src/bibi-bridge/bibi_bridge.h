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
    
public:
    Topic(BibiRegistry* registry, const char* name, size_t capacity = 32) {
        topic = bibi_registry_get_byte_topic(registry, name, capacity);
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
    
    bool receive(void* data, size_t max_len, size_t* actual_len, uint64_t* epoch) {
        if (!topic) return false;
        return bibi_byte_topic_receive(topic, static_cast<uint8_t*>(data), 
                                       max_len, actual_len, epoch) == 0;
    }
    
    template<typename T>
    bool receive(T* msg, uint64_t* epoch = nullptr) {
        size_t actual_len;
        uint64_t ep;
        if (receive(msg, sizeof(T), &actual_len, &ep)) {
            if (epoch) *epoch = ep;
            return actual_len >= sizeof(T);
        }
        return false;
    }
    
    bool peek_latest(void* data, size_t max_len, size_t* actual_len, uint64_t* epoch) {
        if (!topic) return false;
        return bibi_byte_topic_peek_latest(topic, static_cast<uint8_t*>(data),
                                           max_len, actual_len, epoch) == 0;
    }
    
    template<typename T>
    bool peek_latest(T* msg, uint64_t* epoch = nullptr) {
        size_t actual_len;
        uint64_t ep;
        if (peek_latest(msg, sizeof(T), &actual_len, &ep)) {
            if (epoch) *epoch = ep;
            return actual_len >= sizeof(T);
        }
        return false;
    }
    
    bool has_new() {
        if (!topic) return false;
        return bibi_byte_topic_has_new(topic) != 0;
    }
    
    size_t len() {
        if (!topic) return 0;
        return bibi_byte_topic_len(topic);
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
