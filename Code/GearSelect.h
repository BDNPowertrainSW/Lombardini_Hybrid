#ifndef GEARSELECT_H
#define GEARSELECT_H

#ifdef __cplusplus
extern "C" {
#endif

void get_highest_gear_for_tractive_force(double speed, double requested_force, double* gear_res);

void get_air_res(double speed, double* air_res);

void get_rolling_resistance(double speed, double mass, double* rolling_res);

void get_load(double m_engine, double rpm, double* load);

#ifdef __cplusplus
}
#endif

#endif // GEARSELECT_H
