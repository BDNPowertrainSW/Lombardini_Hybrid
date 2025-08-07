
#include <stdio.h>
#include "GearSelect.h"
#include <stddef.h>


void get_highest_gear_for_tractive_force(double speed, double requested_force, double* gear_res) {
    
    const double gear_ratios[] = {3.5, 2.2, 1.4, 1.0};
    const double diff_ratio = 3.9; // Differential ratio
    const int gear_count = 4;

    const double speed_points[] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20};
    const int speed_points_count = 11;

    const double rpm_points[] = {1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000};
    const double max_torque_per_gear[] = {15, 17.13, 20.31, 27.35, 26.2, 24.45, 21.26, 20.64, 19.64};

    double wheel_speed = speed / (2 * 0.24 * 3.141592653589793); // Convert speed from km/h to wheel speed in revolutions per minute (RPM

    for (int i = gear_count - 1; i >= 0; --i) {
        double engine_speed = wheel_speed * diff_ratio * gear_ratios[i];
        
        if (engine_speed > 5000) {
            continue; // Skip gears that would exceed engine speed limit
        }

        // Calculate the torque at the wheels for this gear
        // Interpolate max torque for current engine speed
        double torque = 0.0;
        if (engine_speed <= rpm_points[0]) {
            torque = max_torque_per_gear[0];
        } else if (engine_speed >= rpm_points[speed_points_count - 1]) {
            torque = max_torque_per_gear[speed_points_count - 1];
        } else {
            for (int j = 0; j < speed_points_count - 1; ++j) {
                if (engine_speed >= rpm_points[j] && engine_speed < rpm_points[j + 1]) {
                    double t1 = max_torque_per_gear[j];
                    double t2 = max_torque_per_gear[j + 1];
                    double rpm1 = rpm_points[j];
                    double rpm2 = rpm_points[j + 1];
                    torque = t1 + (t2 - t1) * (engine_speed - rpm1) / (rpm2 - rpm1);
                    break;
                }
            }
        }

        // Calculate tractive force at wheels for this gear
        double tractive_force = torque * gear_ratios[i] * diff_ratio / (0.24); // 0.24 is wheel radius in meters

        if (tractive_force >= requested_force) {
            *gear_res = gear_ratios[i];
            return;
        }
        
    }  
}



void get_air_res(double speed, double* air_res) {
    double air_density = 1.225; // kg/m^3 at sea level and 15 degrees Celsius
    double frontal_area = 1.5; // m^2, typical for a car
    double drag_coefficient = 0.3; // Typical value for a car

    double air_speed = speed / 3.6; // Convert speed from km/h to m/s

    *air_res = 0.5 * air_density * frontal_area * drag_coefficient * (air_speed * air_speed);
}


void get_rolling_resistance(double speed, double mass, double* rolling_res){
    double rolling_coefficient = 0.015; // Typical value for a car tire

    *rolling_res = rolling_coefficient * mass * 9.81; // Force in Newtons
    *rolling_res += 0.01 * speed; // Adding a small speed-dependent term for realism
}
    
void get_load(double m_engine, double rpm, double* load) {
    if (m_engine <= 0 || rpm <= 0) {
        *load = 0.0; // Invalid input, set load to 0
        return;
    }

    // Define RPM and load breakpoints
    const double rpm_points[] = {1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000};
    const int rpm_count = 9;
    const double load_points[] = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    const int load_count = 11;

    // Torque map: rows=load, cols=rpm
    const double torque_map[11][9] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1.1, 2.2, 3.9, 4.8, 3.9, 2.8, 0.4, 0.1, 0.5},
        {1.8, 2.8, 3.5, 5.7, 4.8, 3.65, 1.1, 0.5, 2.2},
        {2.2, 3.6, 4.4, 6.87, 5.76, 4.7, 2, 1, 1.8},
        {2.8, 4.42, 5.4, 8.13, 6.96, 5.8, 2.95, 2.6, 2.7},
        {3.6, 5.44, 6.6, 9.6, 8.38, 7.3, 4.4, 3, 4.1},
        {5.51, 6.79, 8.07, 11.5, 10.27, 8.8, 6.06, 6, 5.8},
        {6.85, 8.3, 9.96, 13.9, 12.68, 11.34, 8.15, 8, 7.8},
        {8.62, 10.4, 12.4, 17.05, 15.56, 14.12, 10.92, 9.6, 10.57},
        {11.07, 12.96, 15.85, 22.1, 22.01, 18.37, 15.51, 14.8, 13.36},
        {15, 17.13, 20.31, 27.35, 26.2, 24.45, 21.26, 20.64, 19.64}
    };

    // Find rpm indices for interpolation
    int rpm_idx = 0;
    for (int i = 0; i < rpm_count - 1; ++i) {
        if (rpm >= rpm_points[i] && rpm < rpm_points[i + 1]) {
            rpm_idx = i;
            break;
        }
    }
    if (rpm >= rpm_points[rpm_count - 1]) {
        rpm_idx = rpm_count - 2;
    }

    // For each load, interpolate torque at given rpm
    double found_load = 0.0;
    for (int l = load_count - 1; l >= 0; --l) {
        double t1 = torque_map[l][rpm_idx];
        double t2 = torque_map[l][rpm_idx + 1];
        double rpm1 = rpm_points[rpm_idx];
        double rpm2 = rpm_points[rpm_idx + 1];
        double torque_at_rpm = t1 + (t2 - t1) * (rpm - rpm1) / (rpm2 - rpm1);

        if (torque_at_rpm >= m_engine) {
            found_load = load_points[l];
            break;
        }
    }
    *load = found_load;

}
