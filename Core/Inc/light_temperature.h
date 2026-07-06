#include "as7341_driver.h"

#define SAMPLE_SIZE 6
#define CHANNEL_COUNT 8
#define PURPLE {118,0, 237}
#define BLUE {0, 0, 140}
#define LIGHT_BLUE {0, 40, 255}
#define GREEN {0, 213, 255}
#define LIGHT_GREEN {31, 255, 0}
#define YELLOW {255, 223, 0}
#define ORANGE {255, 79, 0}
#define RED {255, 0, 0}
#define RGB_COUNT 3

/**
 * @brief Equalizes raw AS7341 channel counts using an inverse-sensitivity
 *        calibration matrix to compensate for silicon's high NIR/Red absorbance,
 *        and normalizes the output into relative percentage weights.
 */
void get_relative_light_colors(uint16_t * ch, float * rel_ch);

/**
 * @brief Maps the 8 relative spectral channels to a simulated 3-channel RGB profile
 *        using hardcoded primary color vectors.
 */
void get_RGB(float * rel_ch, int * RGB_total);

/**
 * @brief Calculates the 3D Euclidean distance between the simulated RGB profile
 *        and a hardcoded Planckian Locus table to find the nearest Color Temperature.
 * @return The matched Color Temperature in Kelvin (e.g., 6000).
 */
uint16_t RGB_to_temperature(int * RGB_total);

/**
 * @brief Filters a window of temperature samples by discarding the sample that
 *        deviates most from the median cluster to eliminate noise spikes.
 */
void remove_outsiders(int * temps, int * cleaned_temps);

/**
 * @brief Top-level execution pipeline for a single AS7341 spectrum readout.
 *        Executes calibration -> RGB mapping -> Temperature matching.
 */
int get_light_temperature(uint16_t * samples);

/**
 * @brief Computes the robust average of a 500ms sliding window (typically 6 samples)
 *        after cleaning outliers.
 */
int get_avg_light_temperature(int * temps);