#include "Copter.h"

/*
 * Init and run calls for div flight mode
 */

// div_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeDIV::run()
{
    // Use the constaant values instead of pilot inputs
    float target_roll = g.div_roll_angle;
    float target_pitch = g.div_pitch_angle;
    float target_yaw = g.div_yaw_angle;
    float pilot_desired_throttle = g.div_throttle;

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Run attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw);

    // Output pilot's throttle without angle boost
    attitude_control->set_throttle_out(pilot_desired_throttle, false, g.throttle_filt);
}
