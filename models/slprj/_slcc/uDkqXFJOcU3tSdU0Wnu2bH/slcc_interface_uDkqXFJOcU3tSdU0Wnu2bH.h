#include "customcode_uDkqXFJOcU3tSdU0Wnu2bH.h"
#ifdef __cplusplus
extern "C" {
#endif


/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
DLL_EXPORT_CC extern const char_T *get_dll_checksum_uDkqXFJOcU3tSdU0Wnu2bH(void);
DLL_EXPORT_CC extern void LambdaPID_Init_uDkqXFJOcU3tSdU0Wnu2bH(LambdaPIDController *pid, real_T kp, real_T ki, real_T kd);
DLL_EXPORT_CC extern real_T LambdaPID_Compute_uDkqXFJOcU3tSdU0Wnu2bH(LambdaPIDController *pid, real_T lambda_setpoint, real_T lambda_measured, real_T dt);
DLL_EXPORT_CC extern real_T calculateCombinedAFR_uDkqXFJOcU3tSdU0Wnu2bH(real_T diesel_stoich_ratio, real_T h2_stoich_ratio, real_T diesel_injected, real_T h2_injected);
DLL_EXPORT_CC extern void Injection_amounts_uDkqXFJOcU3tSdU0Wnu2bH(real_T mass_fraction, real_T air_mass, real_T h2_stoich_ratio, real_T diesel_stoich_ratio, real_T lambda, real_T *s_fuel_trim_diesel, real_T *s_fuel_trim_h2, real_T *h2_injected, real_T *diesel_injected);
DLL_EXPORT_CC extern void airMassCalc_uDkqXFJOcU3tSdU0Wnu2bH(real_T bore, real_T stroke, real_T rpm, real_T volumetric_efficiency, real_T compression_ratio, real_T H2_intake, real_T H2_base_density, real_T h2_iat_correction_factor_scaling, real_T map, real_T iat, real_T *air_mass);
DLL_EXPORT_CC extern void LambdaControl_uDkqXFJOcU3tSdU0Wnu2bH(real_T lambda_reference, real_T mass_fraction, real_T lambda_overwrite, real_T lambda_target_diesel, real_T lambda_target_h2, real_T flex_lambda_scaling, real_T lambda_pid_enable, real_T pid_kp, real_T pid_ki, real_T pid_kd, real_T *flex_lambda);
DLL_EXPORT_CC extern void mass_fraction_calc_uDkqXFJOcU3tSdU0Wnu2bH(real_T energy_fraction, real_T lower_heating_value_h2, real_T lower_heating_value_diesel, real_T *mass_fraction);
DLL_EXPORT_CC extern void start_of_injection_uDkqXFJOcU3tSdU0Wnu2bH(real_T soi_base, real_T dead_time, real_T iat, real_T *soi);
DLL_EXPORT_CC extern void s_fuel_trim_calc_uDkqXFJOcU3tSdU0Wnu2bH(real_T mass_fraction, real_T h2_stoich_ratio, real_T diesel_stoich_ratio, real_T lambda_reference, real_T lambda, real_T *s_fuel_trim_diesel, real_T *s_fuel_trim_h2);
DLL_EXPORT_CC extern void energy_monitor_uDkqXFJOcU3tSdU0Wnu2bH(real_T inj_m_d, real_T inj_m_h2, real_T lower_heating_value_h2, real_T lower_heating_value_diesel, real_T *energy_diesel, real_T *energy_h2, real_T *energy_combined);

/* Function Definitions */
DLL_EXPORT_CC const uint8_T *get_checksum_source_info(int32_T *size);
#ifdef __cplusplus
}
#endif

