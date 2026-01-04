/**
 * TVMC Motion Controller - BiBi-Sync Version
 * Replaces ROS topics with BiBi-Sync topics
 */

#include <iostream>
#include <chrono>
#include "controller.h"

MotionController::MotionController()
{
    std::cout << "[TVMC] Starting BiBi-Sync Motion Controller" << std::endl;
    running = false;

    // load thruster configuration
    config = loadThrusterConfig();

    // initialize the thrust reporter with our registry
    ThrustReporter::init(bridge.get_registry());

    // initialize all controllers,
    // have everything in open loop mode in the beginning.
    // set thrust to 0 for all degrees
    for (int d = 0; d < 6; d++)
    {
        control_modes[d] = OPEN_LOOP_MODE;
        controllers[d].setConstants(1, 1, 1, 0.001);
        controllers[d].setMinMaxLimits(config.spec.min_thrust, config.spec.max_thrust, 
                                       config.spec.min_thrust / 2, config.spec.max_thrust / 2);
        thrust[d] = 0;
    }

    // set PID controllers to angular mode for angles
    controllers[DoF::YAW].setAngular(true);
    controllers[DoF::PITCH].setAngular(true);
    controllers[DoF::ROLL].setAngular(true);

    // set thruster maps for each degree of freedom
    thruster_map[DoF::SURGE] = config.vectors.surge;
    thruster_map[DoF::SWAY] = config.vectors.sway;
    thruster_map[DoF::HEAVE] = config.vectors.heave;
    thruster_map[DoF::YAW] = config.vectors.yaw;
    thruster_map[DoF::PITCH] = config.vectors.pitch;
    thruster_map[DoF::ROLL] = config.vectors.roll;

    // initialize thrust vector to 0
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        thrust_vector.push_back(0);

    // Create BiBi-Sync topics for receiving commands
    topic_command = new bibi::Topic(bridge.get_registry(), bibi::topics::COMMAND, 16);
    topic_control_mode = new bibi::Topic(bridge.get_registry(), bibi::topics::CONTROL_MODE, 16);
    topic_current_point = new bibi::Topic(bridge.get_registry(), bibi::topics::CURRENT_POINT, 16);
    topic_target_point = new bibi::Topic(bridge.get_registry(), bibi::topics::TARGET_POINT, 16);
    topic_pid_constants = new bibi::Topic(bridge.get_registry(), bibi::topics::PID_CONSTANTS, 16);
    topic_pid_limits = new bibi::Topic(bridge.get_registry(), bibi::topics::PID_LIMITS, 16);
    topic_multi_thrust = new bibi::Topic(bridge.get_registry(), bibi::topics::MULTI_THRUST, 16);

    std::cout << "[TVMC] Initialized, topics ready" << std::endl;
}

MotionController::~MotionController()
{
    stop();
    ThrustReporter::shutdown();
    
    delete topic_command;
    delete topic_control_mode;
    delete topic_current_point;
    delete topic_target_point;
    delete topic_pid_constants;
    delete topic_pid_limits;
    delete topic_multi_thrust;
    
    std::cout << "[TVMC] Shutting down" << std::endl;
}

void MotionController::start()
{
    if (running) return;
    
    running = true;
    poll_thread = std::thread(&MotionController::pollMessages, this);
    std::cout << "[TVMC] Started polling thread" << std::endl;
}

void MotionController::stop()
{
    if (!running) return;
    
    running = false;
    online = false;
    if (poll_thread.joinable()) {
        poll_thread.join();
    }
    std::cout << "[TVMC] Stopped polling thread" << std::endl;
}

void MotionController::pollMessages()
{
    while (running && online)
    {
        processMessages();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100Hz polling
    }
}

void MotionController::processMessages()
{
    // Process command messages
    if (topic_command->has_new()) {
        bibi::CommandMsg cmd;
        if (topic_command->receive(&cmd)) {
            if (cmd.command == Cmd::REFRESH)
                this->refresh();
            if (cmd.command == Cmd::RESET_THRUSTERS)
                this->resetAllThrusters();
            if (cmd.command == Cmd::SHUT_DOWN)
                this->online = false;
        }
        topic_command->mark_seen();
    }
    
    // Process control mode changes
    while (topic_control_mode->has_new()) {
        bibi::ControlModeMsg mode;
        if (topic_control_mode->receive(&mode)) {
            this->setControlMode(mode.dof, mode.mode);
        }
        topic_control_mode->mark_seen();
    }
    
    // Process current point updates
    while (topic_current_point->has_new()) {
        bibi::CurrentPointMsg point;
        if (topic_current_point->receive(&point)) {
            this->updateCurrentPoint(point.dof, point.current);
        }
        topic_current_point->mark_seen();
    }

    // Process target point updates
    while (topic_target_point->has_new()) {
        bibi::TargetPointMsg point;
        if (topic_target_point->receive(&point)) {
            this->setTargetPoint(point.dof, point.target);
        }
        topic_target_point->mark_seen();
    }
    
    // Process PID constants updates
    while (topic_pid_constants->has_new()) {
        bibi::PidConstantsMsg constants;
        if (topic_pid_constants->receive(&constants)) {
            this->setPIDConstants(constants.dof, constants.kp, constants.ki, 
                                  constants.kd, constants.acceptable_error, constants.ko);
        }
        topic_pid_constants->mark_seen();
    }
    
    // Process PID limits updates
    while (topic_pid_limits->has_new()) {
        bibi::PidLimitsMsg limits;
        if (topic_pid_limits->receive(&limits)) {
            this->setPIDLimits(limits.dof, limits.output_min, limits.output_max,
                              limits.integral_min, limits.integral_max);
        }
        topic_pid_limits->mark_seen();
    }
    
    // Process multi-thrust commands
    while (topic_multi_thrust->has_new()) {
        bibi::MultiThrustMsg mt;
        if (topic_multi_thrust->receive(&mt)) {
            this->setMultiThrust(mt.surge, mt.sway, mt.heave, mt.roll, mt.pitch, mt.yaw);
        }
        topic_multi_thrust->mark_seen();
    }
}

void MotionController::setControlMode(uint8_t dof, bool mode)
{
    // set control mode for degree of freedom
    control_modes[dof] = mode;

    // if the mode is set to closed loop mode, reset the PID Controller
    if (mode == CLOSED_LOOP_MODE)
        controllers[dof].reset();
    else
        thrust[dof] = 0;
}

void MotionController::setPIDConstants(uint8_t dof, float kp, float ki, float kd, float acceptable_error, float ko)
{
    controllers[dof].setConstants(kp, ki, kd, acceptable_error, ko);
}

void MotionController::setPIDLimits(uint8_t dof, float output_min, float output_max, float integral_min, float integral_max)
{
    controllers[dof].setMinMaxLimits(output_min, output_max, integral_min, integral_max);
}

void MotionController::setTargetPoint(uint8_t dof, float target)
{
    // set target value for controller
    controllers[dof].setTargetValue(target);

    // don't do anything if in open loop mode
    if (control_modes[dof] == OPEN_LOOP_MODE) return;

    // poll output and update thrust
    thrust[dof] = controllers[dof].updateOutput();

    // update thrust values on request
    if (control_modes[dof] == CLOSED_LOOP_MODE)
        MotionController::updateThrustValues();
}

void MotionController::updateCurrentPoint(uint8_t dof, float current)
{
    // update current value for controller
    controllers[dof].setCurrentValue(current);

    // don't do anything if in open loop mode
    if (control_modes[dof] == OPEN_LOOP_MODE) return;

    // poll output and update thrust
    thrust[dof] = controllers[dof].updateOutput();

    // update thrust values on request
    MotionController::updateThrustValues();
}

void MotionController::setMultiThrust(float surge, float sway, float heave, 
                                       float roll, float pitch, float yaw)
{
    // set thrust values without calling update
    if (control_modes[DoF::SURGE] != CLOSED_LOOP_MODE) {
        thrust[DoF::SURGE] = surge;
    }

    if (control_modes[DoF::SWAY] != CLOSED_LOOP_MODE) {
        thrust[DoF::SWAY] = sway;
    }

    if (control_modes[DoF::HEAVE] != CLOSED_LOOP_MODE) {
        thrust[DoF::HEAVE] = heave;
    }

    if (control_modes[DoF::ROLL] != CLOSED_LOOP_MODE) {
        thrust[DoF::ROLL] = roll;
    }

    if (control_modes[DoF::PITCH] != CLOSED_LOOP_MODE) {
        thrust[DoF::PITCH] = pitch;
    }

    if (control_modes[DoF::YAW] != CLOSED_LOOP_MODE) {
        thrust[DoF::YAW] = yaw;
    }
    
    // update after all values are set
    updateThrustValues();
}

void MotionController::resetAllThrusters()
{
    // set thrust at all degrees of freedom to 0
    // will reset all the thrusters at next update
    for (int d = 0; d < 6; d++)
        thrust[d] = 0;
    updateThrustValues();
}

void MotionController::refresh()
{
    // should never really be a need for this unless
    // there is something wrong with the refresh rate
    ThrustReporter::refresh();
}

void MotionController::updateThrustValues()
{
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
    {
        // reset thrust vector to 0
        thrust_vector[i] = 0;

        // add output required by each DoF
        for (int dof = 0; dof < 6; dof++)
            thrust_vector[i] += (thrust[dof] * thruster_map[dof][i]);

        // clamp thrust
        thrust_vector[i] = limitToRange(thrust_vector[i], config.spec.min_thrust, config.spec.max_thrust);
    }

    // report thrust to the thrust reporter
    float *thrust_array = &thrust_vector[0];
    ThrustReporter::writeThrusterValues(thrust_array);
}

float MotionController::limitToRange(float value, float minimum, float maximum)
{
    if (value > maximum)
        return maximum;
    if (value < minimum)
        return minimum;
    return value;
}

int main(int argc, char **argv)
{
    std::cout << "[TVMC] BiBi-Sync Motion Controller starting..." << std::endl;
    
    // make a motion controller instance
    auto m = new MotionController();
    m->start();

    // keep the main loop running
    while (m->online)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // shut down the motion controller
    delete m;

    std::cout << "[TVMC] Goodbye :)" << std::endl;
    return 0;
}
