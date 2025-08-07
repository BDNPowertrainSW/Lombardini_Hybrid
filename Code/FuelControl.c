#include <stddef.h>
#include "FuelControl.h"
#include <math.h>


// Initialize the PID controller
void LambdaPID_Init(LambdaPIDController* pid, double kp, double ki, double kd) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
}

// Compute PID output for Lambda control
double LambdaPID_Compute(LambdaPIDController* pid, double lambda_setpoint, double lambda_measured, double dt) {
    if (!pid || dt <= 0.0) return 0.0;
    double error = lambda_setpoint - lambda_measured;
    pid->integral += error * dt;
    double derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * error + pid->kd * error;
}


// This function initializes the PID controller if not already done and computes the PID output
double LambdaPID_Update(double setpoint, double measured, double kp, double ki, double kd, double dt) {
    static LambdaPIDController pid;
    static int initialized = 0;

    if (!initialized) {
        LambdaPID_Init(&pid, kp, ki, kd);
        initialized = 1;
    }

    return LambdaPID_Compute(&pid, setpoint, measured, dt);
}


/* Function implementation */
double calculateCombinedAFR(double diesel_stoich_ratio, double h2_stoich_ratio, double diesel_injected, double h2_injected)
{
    if (diesel_stoich_ratio <= 0 || h2_stoich_ratio <= 0 || diesel_injected < 0 || h2_injected < 0) {
        return 0.0; // Invalid input, set combined AFR to 0
    }
    return(((diesel_stoich_ratio * diesel_injected) + (h2_stoich_ratio * h2_injected)) / (h2_injected + diesel_injected));
}


void Injection_amounts(double mass_fraction, double air_mass, double h2_stoich_ratio, double diesel_stoich_ratio, double lambda, double s_fuel_trim_diesel, double s_fuel_trim_h2, double* h2_injected, double* diesel_injected)
{
    if (mass_fraction < 0 || air_mass <= 0 || h2_stoich_ratio <= 0 || diesel_stoich_ratio <= 0 || lambda <= 0) {
        return 0.0; // Invalid input, return 0
    }

    double comb_afr = diesel_stoich_ratio * (1 - mass_fraction) + h2_stoich_ratio * mass_fraction; 


    double overall_fuel_mass = air_mass / (comb_afr * lambda); // Total fuel mass based on air mass and combined AFR
    // Calculate the amount of Hydrogen injected based on the mass fraction and stoichiometric ratios
    *h2_injected = overall_fuel_mass * mass_fraction;

   // TODO: Apply any corrections necessary


    // Calculate the amount of Diesel injected based on the remaining air mass after Hydrogen injection
    *diesel_injected = overall_fuel_mass * (1 - mass_fraction);

    //TODO: Apply any corrections necessary
    *diesel_injected += s_fuel_trim_diesel; // Apply fuel trim to diesel injected amount
    *h2_injected += s_fuel_trim_h2; // Apply fuel trim to hydrogen injected amount
}

void s_fuel_trim_calc(double mass_fraction, double h2_stoich_ratio, 
    double diesel_stoich_ratio, double lambda_reference, double lambda, double* s_fuel_trim_diesel, double* s_fuel_trim_h2) 
{
    if (mass_fraction < 0 || h2_stoich_ratio <= 0 || diesel_stoich_ratio <= 0 || lambda <= 0 ) {
        return 0.0; // Invalid pointers, return 0
    }

    double lambda_diff = lambda - lambda_reference;
    double gain = 0.1 * lambda_diff; // Adjust gain as necessary
    if (gain < 0.0) gain = 0.0;
    if (gain > 1.0) gain = 1.0;

    // Calculate the fuel trim for Diesel and Hydrogen based on the mass fraction and stoichiometric ratios
    *s_fuel_trim_diesel = (1 - mass_fraction) * h2_stoich_ratio * gain * lambda_diff;
    *s_fuel_trim_h2 = mass_fraction *  h2_stoich_ratio * gain * lambda_diff;
}

// Function to calculate the air mass based on engine parameters
// Result is valid in mm3 for a single cylinder
void airMassCalc(double bore, double stroke, double rpm, double volumetric_efficiency, double compression_ratio, double H2_intake, double H2_base_density, double h2_iat_correction_factor_scaling , double map, double iat, double* air_mass) {
    if (bore <= 0 || stroke <= 0 || rpm <= 0 || volumetric_efficiency <= 0 || compression_ratio <= 1 || H2_intake < 0 || H2_base_density <= 0 || h2_iat_correction_factor_scaling <= 0 || map <= 0 || iat <= 0) {
    *air_mass = 0.0; // Invalid input, set air mass to 0
        return;
    }

    double h2_rho_correction_factor = 298 / iat;
    double h2_volume = H2_intake / (h2_rho_correction_factor * H2_base_density * h2_iat_correction_factor_scaling); // Adjusted for intake conditions;

    // Calculate displacement in mm3
    double base_displacement = (bore/2) * (bore/2) * 3.141592653589793 * stroke; // Volume in mm^3
    double displacement_w_compression_ratio = (1 + 1/compression_ratio) * base_displacement; // Adjusted for compression ratio
    double displacement_w_volumetric_efficiency = displacement_w_compression_ratio * (volumetric_efficiency/100);
    double displacement_w_h2_displacement = displacement_w_volumetric_efficiency - h2_volume; // Adjusted for Hydrogen intake

    // 287 is the adiabatic gas constant (R)
    *air_mass = ((map / (iat * 287)) * displacement_w_h2_displacement) * 0.001; //Calculate air mass in mg for single cylinder.
}


// TEST CHANGE
// Lambda control function, calculates the flex lambda target based on the mass fraction and target lambda LUTs, and applies PID control if necessary.
void LambdaControl(double lambda_reference, double mass_fraction, double lambda_overwrite, double lambda_target_diesel, double lambda_target_h2, double flex_lambda_scaling, double lambda_pid_enable, double pid_kp, double pid_ki, double pid_kd, double* flex_lambda) {
    if (lambda_reference <= 0 || mass_fraction < 0 || lambda_target_diesel <= 0 || lambda_target_h2 <= 0 || flex_lambda_scaling <= 0) {
        *flex_lambda = 3; // Invalid input, set flex lambda to 3
        return;
    }

    if (lambda_overwrite > 0) {
        *flex_lambda = lambda_overwrite; // Use the overwritten lambda value
    } else {
        // Calculate the flex lambda based on the mass fraction and target lambdas
        // Scaling is implemented, possibly necessary for calibration
        *flex_lambda = (lambda_target_diesel * (1 - mass_fraction) + lambda_target_h2 * mass_fraction * flex_lambda_scaling) ;

        if ( lambda_pid_enable > 0) {
            *flex_lambda += LambdaPID_Update(*flex_lambda, lambda_reference, pid_kp, pid_ki, pid_kd, 0.005); // Update with PID control
        }
    }

}

void mass_fraction_calc(double energy_fraction, double lower_heating_value_h2, double lower_heating_value_diesel, double *mass_fraction)
{
    if (lower_heating_value_h2 <= 0.0 || lower_heating_value_diesel <= 0.0 || energy_fraction <= 0.0 || energy_fraction > 1.0) {
        *mass_fraction = 0.0;
        return;
    }
    // Avoid division if energy_fraction is 1.0 (would cause division by zero)
    double denom = lower_heating_value_h2 * (1.0 - energy_fraction);
    if (denom <= 0.0) {
        *mass_fraction = 0.0;
        return;
    }
    double tmp = lower_heating_value_diesel * energy_fraction / denom;
    *mass_fraction = tmp / (tmp + 1.0);
}



void start_of_injection(double soi_base, double dead_time, double iat,  double* soi)
{
    *soi = soi_base + dead_time; // Initialize SOI with the base value

    //TODO: Apply any corrections necessary based on IAT or other parameters
}


void energy_monitor(double inj_m_d, double inj_m_h2, double lower_heating_value_h2, double lower_heating_value_diesel, double* energy_diesel, double* energy_h2, double* energy_combined) {
    if (lower_heating_value_h2 <= 0.0 || lower_heating_value_diesel <= 0.0 || inj_m_d < 0.0 || inj_m_h2 < 0.0) {
        *energy_diesel = inj_m_d * lower_heating_value_diesel / 1000;
        *energy_h2 = inj_m_h2 * lower_heating_value_h2 / 1000;
        *energy_combined = *energy_diesel + *energy_h2; // Total energy in J
        return;
    }

    // Calculate the energy from Diesel and Hydrogen injections
    *energy_diesel = inj_m_d * lower_heating_value_diesel; // Energy from Diesel in J
    *energy_h2 = inj_m_h2 * lower_heating_value_h2; // Energy from Hydrogen in J

    // Calculate the combined energy
    *energy_combined = *energy_diesel + *energy_h2; // Total energy in J
}




