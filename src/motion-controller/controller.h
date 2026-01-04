/**
 * TVMC Motion Controller - BiBi-Sync Version
 * Replaces ROS topics with BiBi-Sync topics
 */

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "../PID-Controller/PID_controller.h"
#include "../reporters/thrust.h"
#include "../thruster-config/thruster_config.h"
#include "../bibi-bridge/bibi_bridge.h"
#include <vector>
#include <thread>
#include <atomic>

#define CLOSED_LOOP_MODE 0
#define OPEN_LOOP_MODE 1

// DoF constants (matching Python enums)
namespace DoF {
    constexpr uint8_t SURGE = 0;
    constexpr uint8_t SWAY = 1;
    constexpr uint8_t HEAVE = 2;
    constexpr uint8_t ROLL = 3;
    constexpr uint8_t PITCH = 4;
    constexpr uint8_t YAW = 5;
}

// Command constants
namespace Cmd {
    constexpr uint8_t RESET_THRUSTERS = 0;
    constexpr uint8_t REFRESH = 1;
    constexpr uint8_t SHUT_DOWN = 2;
}

class MotionController
{
private:
    // BiBi-Sync bridge
    bibi::Bridge bridge;
    
    // BiBi-Sync topics for receiving commands
    bibi::Topic* topic_command;
    bibi::Topic* topic_control_mode;
    bibi::Topic* topic_current_point;
    bibi::Topic* topic_target_point;
    bibi::Topic* topic_pid_constants;
    bibi::Topic* topic_pid_limits;
    bibi::Topic* topic_multi_thrust;
    
    // Polling thread
    std::thread poll_thread;
    std::atomic<bool> running;

    // Thrust in each degree of motion
    float thrust[6];

    // Control modes for each degree of motion
    float control_modes[6];

    // PID controllers for each degree of freedom
    PIDController controllers[6];

    // The MotionController's copy of the Thruster config
    ThrusterConfig config;

    // The thruster map for each degree of freedom
    std::vector<float> thruster_map[6];

    // Final thrust vector for with thrust for each vector
    std::vector<float> thrust_vector;

public:
    bool online = true;

    MotionController();
    ~MotionController();

    /**
     * Start polling for messages
     */
    void start();
    
    /**
     * Stop polling
     */
    void stop();

    /**
     * Changes control mode for each DoF
     * 
     * @param dof The Degree of Freedom
     * @param mode The Control Mode
    */
    void setControlMode(uint8_t dof, bool mode);

    /**
     * Adjusts PID constants for each DoF
     * 
     * @param dof The Degree of Freedom
     * @param kp Propotional Constant
     * @param ki Integral Constant
     * @param kd Derivative Constant
     * @param acceptable_error Minimum error required for the controller to perform corrections.
    */
    void setPIDConstants(uint8_t dof, float kp, float ki, float kd, float acceptable_error, float ko = 0);

    /**
     * Adjusts PID limits in each DoF
     * 
     * @param dof The Degree of Freedom
     * @param output_min Minimum output thrust from the controller
     * @param output_max Maximum output thrust from the controller
     * @param integral_min Minimum integral contribution to thrust from the controller
     * @param integral_max Maximum integral contribution  thrust from the controller
    */
    void setPIDLimits(uint8_t dof, float output_min, float output_max, float integral_min, float integral_max);

    /**
     * Sets target values in each DoF
     * Works only if closed loop control is enabled
     * 
     * @param dof The Degree of Freedom
     * @param target Target for the PID Controller
    */
    void setTargetPoint(uint8_t dof, float target);
    
    /**
     * Sets current values in each DoF
     * Works only if closed loop control is enabled
     * 
     * @param dof The Degree of Freedom
     * @param target Current value for the PID Controller
    */
    void updateCurrentPoint(uint8_t dof, float current);

    /**
     * Sets multi-thrust values for all DoFs
     */
    void setMultiThrust(float surge, float sway, float heave, float roll, float pitch, float yaw);

    /**
     * Resets all thrusters to zero
    */
    void resetAllThrusters();
    
    /**
     * Requests an immediate refresh fromt the Thrust Reporter
    */
    void refresh();

    /**
     * Updates all thrust values to the Thrust Reporter
    */
    void updateThrustValues();
    
    /**
     * Get the BiBi-Sync registry for external use
     */
    BibiRegistry* getRegistry() { return bridge.get_registry(); }

private:
    /**
     * Poll for incoming messages
     */
    void pollMessages();
    
    /**
     * Process incoming messages
     */
    void processMessages();

    /**
     * Simple clamping function
    */
    float limitToRange(float value, float minimum, float maximum);
};

#endif