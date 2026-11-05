/*
 * C interface of the EV-DynaSim engine.
 *
 * Why a C interface? C++ has no stable binary interface (ABI): name mangling,
 * std::vector layout and exception handling differ between compilers and
 * versions. Plain C functions with plain C types can be called from almost
 * anything: Python (ctypes), MATLAB (loadlibrary), LabVIEW, C#, an FMU
 * wrapper. Rules followed here:
 *   - only C types cross the boundary (double*, size_t, char*, structs of doubles)
 *   - the caller owns all memory; the library never returns heap pointers
 *   - no C++ exception crosses the boundary: errors are return codes + message
 */
#ifndef EVDYNASIM_C_H
#define EVDYNASIM_C_H

#include <stddef.h>

#if defined(_WIN32)
#  if defined(EVDS_BUILDING_DLL)
#    define EVDS_API __declspec(dllexport)
#  else
#    define EVDS_API __declspec(dllimport)
#  endif
#else
#  define EVDS_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Field order must match EvdsSummary in src/evdynasim/cpp_engine.py. */
typedef struct evds_summary {
    double duration_s, distance_km, mean_speed_kmh, max_speed_kmh;
    double battery_energy_kwh, consumption_kwh_per_100km, chemical_energy_kwh;
    double regen_energy_kwh, regen_curtailed_kwh;
    double aero_kwh, rolling_kwh, grade_kwh, kinetic_change_kwh;
    double gearbox_loss_kwh, motor_loss_kwh, friction_brake_kwh, aux_kwh, battery_loss_kwh;
    double peak_battery_power_kw, peak_regen_power_kw, max_current_a, min_terminal_voltage_v;
    double initial_soc, final_soc, max_temp_c, final_temp_c;
    int torque_limited_steps, battery_limited_steps;
} evds_summary;

enum { EVDS_OK = 0, EVDS_ERROR = 1 };

EVDS_API const char* evds_version(void);

/*
 * Simulate a drive cycle given as n samples.
 *   config_path      vehicle YAML file
 *   time_s, speed_mps, grade_pct   input arrays of length n (n >= 2)
 *   battery_power_w  output, length n-1, may be NULL
 *   soc, temp_c      outputs, length n, may be NULL
 *   summary          output, may be NULL
 *   error_msg        buffer for an error message, may be NULL
 * Returns EVDS_OK or EVDS_ERROR.
 */
EVDS_API int evds_simulate(const char* config_path, const double* time_s, const double* speed_mps,
                           const double* grade_pct, size_t n, double* battery_power_w, double* soc,
                           double* temp_c, evds_summary* summary, char* error_msg, size_t error_len);

#ifdef __cplusplus
}
#endif

#endif /* EVDYNASIM_C_H */
