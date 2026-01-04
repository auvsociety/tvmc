/**
 * PWM Reporter - BiBi-Sync Version
 * Converts thrust values to PWM and sends to STM32 via BiBi-Sync
 */

#ifndef PWM_REPORTER_H
#define PWM_REPORTER_H

extern "C" {
#include "bibi_sync.h"
}

#include "../thruster-config/thruster_config.h"
#include <libInterpolate/Interpolate.hpp>

#define PWM_REPORTING_FREQ 10

namespace PWMReporter
{
    class Thruster
    {
        _1D::MonotonicInterpolator<float> &interpolater;

        float clamp;
        float max_thrust;
        float min_thrust;

    public:
        Thruster(_1D::MonotonicInterpolator<float> &interpolater, float min_thrust, float max_thrust);

        // Computes the PWM value for the required thrust (in kgf).
        int compute_pwm(float thrust);
    };
    
    /**
     * Initialize PWM reporter with BiBi-Sync registry
     */
    void init(BibiRegistry* registry);
    
    /**
     * Shutdown PWM reporter
     */
    void shutdown();
    
    /**
     * Start the PWM reporter main loop
     */
    void run();
};

#endif