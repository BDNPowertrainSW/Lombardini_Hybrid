#include <stdio.h>
#define LOAD_TABLE_SIZE 101

// Calculates the required fuel amount for a given load percentage (0-100)
// and maximum torque (Nm). Returns fuel amount in arbitrary units.
void calculate_fuel_for_torque(double load_percent, double max_torque, double rpm, double energy_fraction, double* h2_mass, double* diesel_mass)
{
    double load_to_torque_table[LOAD_TABLE_SIZE];
    for (int i = 0; i < LOAD_TABLE_SIZE; ++i)
    {
        // Linear mapping: 0% -> 0Nm, 100% -> 30Nm
        load_to_torque_table[i] = (i / 100.0) * 30.0;
    }
    double requested_torque = load_to_torque_table[(int)load_percent];

    double fuel_energy = (requested_torque * rpm * (2 * 3.141592653589793 / 60.0))/0.38; // Energy in Joules
    double mass_fraction = 0.0;

    // Avoid division if energy_fraction is 1.0 (would cause division by zero)
    double denom = 120000 * (1.0 - energy_fraction);
    if (denom <= 0.0) {
        mass_fraction = 0.0;
        return;
    }
    double tmp = 42800 * energy_fraction / denom;
    mass_fraction = tmp / (tmp + 1.0);

    double h2_energy = mass_fraction * fuel_energy; // Energy from Hydrogen in Joules
    double diesel_energy = (1.0 - mass_fraction) * fuel_energy; // Energy

    *h2_mass = h2_energy / 120000; // Mass of Hydrogen in kg
    *diesel_mass = diesel_energy / 42800; // Mass of Diesel in kg

}




