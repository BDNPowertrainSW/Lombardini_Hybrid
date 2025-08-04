#ifndef FUELCONTROL_H
#define FUELCONTROL_H

// FuelControl.h
// Provides function declarations for fuel control logic, including AFR calculations.

typedef struct {
    double kp;
    double ki;
    double kd;
    double prev_error;
    double integral;
} LambdaPIDController;

void LambdaPID_Init(LambdaPIDController* pid, double kp, double ki, double kd);

double LambdaPID_Compute(LambdaPIDController* pid, double lambda_setpoint, double lambda_measured, double dt);

/**
 * @brief Calculates the combined Air-Fuel Ratio (AFR) for dual fuel operation.
 *
 * @param diesel_stoich_ratio   Stochiometric ratio for Diesel fuel
 * @param h2_stoich_ratio       Stochiometric ratio for Hydrogen fuel
 * @param diesel_injected       Amount of Diesel injected
 * @param h2_injected           Amount of Hydrogen injected
 * @return combined_afr          Combined AFR value
 */

// Calculates the combined Air-Fuel Ratio (AFR) for dual fuel operation.
double calculateCombinedAFR(double diesel_stoich_ratio, double h2_stoich_ratio, double diesel_injected, double h2_injected);

/**
 * @brief Calculates the amount of fuel injected based on mass fraction and stoichiometric ratios.
 *
 * @param mass_fraction         Mass fraction of Hydrogen in the mixture
 * @param air_mass              Total air mass available for combustion
 * @param h2_stoich_ratio       Stochiometric ratio for Hydrogen fuel
 * @param diesel_stoich_ratio   Stochiometric ratio for Diesel fuel
 * @param lambda                Lambda value for the mixture
 * @param h2_injected           Pointer to store the amount of Hydrogen injected
 * @param diesel_injected       Pointer to store the amount of Diesel injected
 */

void Injection_amounts(double mass_fraction, double air_mass, double h2_stoich_ratio, double diesel_stoich_ratio, double lambda, double* s_fuel_trim_diesel, double* s_fuel_trim_h2, double* h2_injected, double* diesel_injected);

/**
 * @brief Calculates the airmass entering the engine using physical and sensor parameters.
 *
 * This function computes the mass of air supplied to the engine cylinder per cycle,
 * factoring in engine geometry, operating conditions, and hydrogen intake characteristics.
 * It uses sensor readings (MAP, IAT), engine speed, volumetric efficiency, compression ratio,
 * and hydrogen-specific parameters to determine the airmass.
 *
 * @param bore Cylinder bore diameter.
 * @param stroke Cylinder stroke length.
 * @param rpm Engine speed in revolutions per minute.
 * @param volumetric_efficiency Engine volumetric efficiency.
 * @param compression_ratio Engine compression ratio.
 * @param H2_intake Hydrogen intake amount.
 * @param H2_base_density Base density of hydrogen.
 * @param h2_iat_correction_factor_scaling Scaling factor for hydrogen IAT correction.
 * @param map Manifold absolute pressure sensor value.
 * @param iat Intake air temperature sensor value.
 * @param air_mass Pointer to store the calculated air mass value.
 */

void airMassCalc(double bore, double stroke, double rpm, double volumetric_efficiency, double compression_ratio, double H2_intake, double H2_base_density, double h2_iat_correction_factor_scaling , double map, double iat, double* air_mass);

/**
 * @brief Performs lambda control logic for dual fuel operation.
 *
 * This function manages the lambda (air-fuel ratio) control for a dual fuel engine,
 * adjusting the mixture based on reference values, mass fraction, PID parameters, and scaling factors.
 * It computes the flexible lambda value for the fuel mixture, supporting both open-loop and closed-loop control.
 *
 * @param lambda_reference      Reference lambda value for the mixture
 * @param mass_fraction         Mass fraction of Hydrogen in the mixture
 * @param lambda_overwrite      Overwrite value for lambda (if applicable)
 * @param lambda_target_diesel  Target lambda for Diesel fuel
 * @param lambda_target_h2      Target lambda for Hydrogen fuel
 * @param flex_lambda_scaling   Scaling factor for flexible lambda adjustment
 * @param lambda_pid_enable     Enable flag for PID-based lambda control
 * @param pid_kp                Proportional gain for PID controller
 * @param pid_ki                Integral gain for PID controller
 * @param pid_kd                Derivative gain for PID controller
 * @param flex_lambda           Pointer to store the computed flexible lambda value
 */

void LambdaControl(double lambda_reference, double mass_fraction, double lambda_overwrite, double lambda_target_diesel, double lambda_target_h2, double flex_lambda_scaling, double lambda_pid_enable, double pid_kp, double pid_ki, double pid_kd, double* flex_lambda);

/**
 * @brief Calculates the mass fraction of hydrogen in a dual fuel mixture based on energy fraction and lower heating values.
 *
 * This function computes the mass fraction of hydrogen in the mixture using the provided energy fraction
 * and the lower heating values (LHV) of hydrogen and diesel. The result is stored in the mass_fraction pointer.
 *
 * @param energy_fraction           Fraction of energy provided by hydrogen
 * @param lower_heating_value_h2    Lower heating value of hydrogen
 * @param lower_heating_value_diesel Lower heating value of diesel
 * @param mass_fraction             Pointer to store the calculated mass fraction of hydrogen
 */

void mass_fraction_calc(double energy_fraction, double lower_heating_value_h2, double lower_heating_value_diesel, double *mass_fraction);


/**
 * @brief Starts the injection process.
 *
 * This function calculates the start of injection timing based on a base look up table and minor corrections.
 *
 * @param soi_base   Base start of injection timing coming from the calibration table
 * @param dead_time  Injector dead time coming from the table (u_bat and common rail pressure dependent
 * @param iat        Intake air temperature
 * @param soi        Pointer to store the calculated start of injection value
 */

void start_of_injection(double soi_base, double dead_time, double iat,  double* soi);

void s_fuel_trim_calc(double mass_fraction, double h2_stoich_ratio, double diesel_stoich_ratio, double lambda_reference, double lambda, double* s_fuel_trim_diesel, double* s_fuel_trim_h2);


void energy_monitor(double inj_m_d, double inj_m_h2, double lower_heating_value_h2, double lower_heating_value_diesel, double* energy_diesel, double* energy_h2, double* energy_combined);

/**
 * @brief 
 *
 * This function integrates all fuel control steps: mass fraction calculation, air mass calculation,
 * lambda control, and injection amount computation. It outputs diesel and hydrogen injection amounts,
 * diesel injection duration, and start of injection timing.
 *
 * @param energy_fraction           Fraction of energy provided by hydrogen
 * @param lower_heating_value_h2    Lower heating value of hydrogen
 * @param lower_heating_value_diesel Lower heating value of diesel
 * @param bore                      Cylinder bore diameter
 * @param stroke                    Cylinder stroke length
 * @param rpm                       Engine speed in revolutions per minute
 * @param volumetric_efficiency     Engine volumetric efficiency
 * @param compression_ratio         Engine compression ratio
 * @param H2_intake                 Hydrogen intake amount
 * @param H2_base_density           Base density of hydrogen
 * @param h2_iat_correction_factor_scaling Scaling factor for hydrogen IAT correction
 * @param map                       Manifold absolute pressure sensor value
 * @param iat                       Intake air temperature sensor value
 * @param lambda_reference          Reference lambda value for the mixture
 * @param lambda_overwrite          Overwrite value for lambda (if applicable)
 * @param lambda_target_diesel      Target lambda for Diesel fuel
 * @param lambda_target_h2          Target lambda for Hydrogen fuel
 * @param flex_lambda_scaling       Scaling factor for flexible lambda adjustment
 * @param lambda_pid_enable         Enable flag for PID-based lambda control
 * @param pid_kp                    Proportional gain for PID controller
 * @param pid_ki                    Integral gain for PID controller
 * @param pid_kd                    Derivative gain for PID controller
 * @param soi_base                  Base start of injection timing
 * @param dead_time                 Injector dead time
 * @param diesel_stoich_ratio       Stochiometric ratio for Diesel fuel
 * @param h2_stoich_ratio           Stochiometric ratio for Hydrogen fuel
 * @param diesel_injector_flow_rate Diesel injector flow rate (mg/ms)
 * @param diesel_injection_duration Pointer to store diesel injection duration
 * @param diesel_injected           Pointer to store diesel injection amount
 * @param h2_injected               Pointer to store hydrogen injection amount
 * @param soi                       Pointer to store start of injection value
 */

/*
void FuelControl_Run(
    double energy_fraction,
    double lower_heating_value_h2,
    double lower_heating_value_diesel,
    double bore,
    double stroke,
    double rpm,
    double volumetric_efficiency,
    double compression_ratio,
    double H2_intake,
    double H2_base_density,
    double h2_iat_correction_factor_scaling,
    double map,
    double iat,
    double lambda_reference,
    double lambda_overwrite,
    double lambda_target_diesel,
    double lambda_target_h2,
    double flex_lambda_scaling,
    double lambda_pid_enable,
    double pid_kp,
    double pid_ki,
    double pid_kd,
    double soi_base,
    double dead_time,
    double diesel_stoich_ratio,
    double h2_stoich_ratio,
    double diesel_injector_flow_rate,
    double* diesel_injection_duration,
    double* diesel_injected,
    double* h2_injected,
    double* soi
);

*/



#endif // FUELCONTROL_H
