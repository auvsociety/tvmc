/**
 * Thrust Reporter - BiBi-Sync Version
 * Publishes thrust values via BiBi-Sync topics
 */

#ifndef THRUST_REPORTER_H
#define THRUST_REPORTER_H

extern "C" {
#include "bibi_sync.h"
}

#define THRUST_REPORT_RATE_US 100000

namespace ThrustReporter
{
    /**
     *  Initializes the Thrust Reporter with BiBi-Sync registry
     */
    void init(BibiRegistry* registry);
    
    /**
     * Reports the corresponding thrust values for the provided thrust vector.
     * 
     * @param thrust_vector List of float values representing the thrust of each thruster.
    */
    void report(float* thrust_vector);

    /**
     * Kills the Thrust Reporter
    */
    void kill();

    /**
     * Force re-publishes the thrust values.
    */
    void refresh();

    /**
     * Alias for kill().
     * Provided for backwards-compatibility with ThrusterController.
    */
    void shutdown();

    /**
     * Alias for report()
     * Provided for backwards-compatibility with ThrusterController.
     * 
     * @param thrust_vector List of float values representing the thrust of each thruster.
    */
    void writeThrusterValues(float* thrust_vector);
}

#endif
