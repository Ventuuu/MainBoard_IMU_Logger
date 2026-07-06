#include "mic_metrics.h"
#include <math.h>
#include "main.h"

typedef struct
{
    double b0;
    double b1;
    double b2;
    double a1;
    double a2;
} AudioBiquadCoefficients;

typedef struct
{
    double s1;
    double s2;
} AudioBiquadState;

typedef struct
{
    uint32_t window_index;
    uint32_t window_start_ms;
    uint32_t sample_count;

    double mean_counts;
    double rms_zero_mean_counts;
    double rms_zero_mean_dbfs;

    uint32_t absolute_peak_counts;
    double peak_dbfs;

    uint32_t clipped_sample_count;
    double clipped_sample_percentage;

    double a_weighted_rms_counts;
    double a_weighted_rms_dbfs;
    double estimated_laeq_dba;

    uint8_t environment_class;
    uint8_t audio_flags;
    uint8_t acquisition_valid;
    uint8_t a_weighting_valid;
    uint8_t record_valid;

    uint8_t complete;
    uint8_t valid;
} AudioBasicFeatureDebug;

/*
 * Denominator convention: 1 + a1*z^-1 + a2*z^-2. The DF-II transposed
 * state updates therefore subtract a1*y and a2*y.
 * Jens Hee, "A-weighting filter for 44.1 and 48 kHz sampling", 2019.
 */
static const AudioBiquadCoefficients audio_a_weighting_biquads[3] =
{
    {0.96525096525, -1.34730163086, 0.38205066561,
     -1.34730722798, 0.34905752979},
    {0.94696969696, -1.89393939393, 0.94696969696,
     -1.89387049481, 0.89515976917},
    {0.64666542810, -0.38362237137, -0.26304305672,
     -1.34730722798, 0.34905752979}
};


static AudioBasicFeatureDebug Audio_ComputeBasicFeatures(
        const int16_t *samples,
        uint32_t sample_count)
{
    AudioBasicFeatureDebug features = {0};
    int64_t sample_sum = 0;
    double centered_energy = 0.0;
    uint32_t absolute_peak = 0U;
    uint32_t clipped_count = 0U;

    features.sample_count = sample_count;
    features.rms_zero_mean_dbfs = -INFINITY;
    features.peak_dbfs = -INFINITY;
    features.a_weighted_rms_dbfs = -INFINITY;
    features.estimated_laeq_dba = -INFINITY;
    features.environment_class = 0;

    if ((samples == NULL) || (sample_count == 0U))
    {
        return features;
    }

    for (uint32_t i = 0U; i < sample_count; i++)
    {
        sample_sum += samples[i];
    }

    features.mean_counts = (double)sample_sum / (double)sample_count;

    for (uint32_t i = 0U; i < sample_count; i++)
    {
        int32_t sample = samples[i];
        uint32_t magnitude = (sample < 0) ? (uint32_t)(-sample) : (uint32_t)sample;
        double centered = (double)sample - features.mean_counts;

        centered_energy += centered * centered;

        if (magnitude > absolute_peak)
        {
            absolute_peak = magnitude;
        }

        if ((sample == INT16_MIN) || (sample == INT16_MAX))
        {
            clipped_count++;
        }
    }

    features.rms_zero_mean_counts = sqrt(centered_energy / (double)sample_count);
    features.absolute_peak_counts = absolute_peak;
    features.clipped_sample_count = clipped_count;
    features.clipped_sample_percentage =
            (100.0 * (double)clipped_count) / (double)sample_count;

    if (features.rms_zero_mean_counts > 0.0)
    {
        features.rms_zero_mean_dbfs =
                20.0 * log10(features.rms_zero_mean_counts / 32768.0);
    }

    if (absolute_peak > 0U)
    {
        features.peak_dbfs = 20.0 * log10((double)absolute_peak / 32768.0);
    }

    return features;
}

static uint8_t Audio_ComputeAWeightedFeatures(
        const int16_t *samples,
        uint32_t sample_count,
        uint32_t sample_rate_hz,
        double mean_counts,
        double *rms_counts,
        double *rms_dbfs)
{
    AudioBiquadState states[3] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    double weighted_energy = 0.0;
    double mean_square;

    if ((rms_counts == NULL) || (rms_dbfs == NULL))
    {
        return 0U;
    }

    *rms_counts = 0.0;
    *rms_dbfs = -INFINITY;

    if ((samples == NULL) || (sample_count == 0U) ||
        (sample_rate_hz != 48000) ||
        !isfinite(mean_counts))
    {
        return 0U;
    }

    /* Each one-second window is independent, so all biquad states start at zero. */
    for (uint32_t i = 0U; i < sample_count; i++)
    {
        double section_input = (double)samples[i] - mean_counts;

        for (uint32_t section = 0U; section < 3U; section++)
        {
            const AudioBiquadCoefficients *coefficients =
                    &audio_a_weighting_biquads[section];
            double output = coefficients->b0 * section_input + states[section].s1;
            double next_s1 = coefficients->b1 * section_input -
                             coefficients->a1 * output + states[section].s2;
            double next_s2 = coefficients->b2 * section_input -
                             coefficients->a2 * output;

            if (!isfinite(output) || !isfinite(next_s1) || !isfinite(next_s2))
            {
                return 0U;
            }

            states[section].s1 = next_s1;
            states[section].s2 = next_s2;
            section_input = output;
        }

        weighted_energy += section_input * section_input;
        if (!isfinite(weighted_energy) || (weighted_energy < 0.0))
        {
            return 0U;
        }
    }

    mean_square = weighted_energy / (double)sample_count;
    if (!isfinite(mean_square) || (mean_square < 0.0))
    {
        return 0U;
    }

    *rms_counts = sqrt(mean_square);
    if (!isfinite(*rms_counts) || (*rms_counts <= 0.0))
    {
        *rms_counts = 0.0;
        return 0U;
    }

    *rms_dbfs = 20.0 * log10(*rms_counts / 32768.0);
    if (!isfinite(*rms_dbfs))
    {
        *rms_dbfs = -INFINITY;
        return 0U;
    }

    return 1U;
}

static uint8_t Audio_ClassifyEnvironment(double estimated_laeq_dba)
{
    if (!isfinite(estimated_laeq_dba))
    {
        return 0;
    }

    if (estimated_laeq_dba < 40.0) return 1;
    if (estimated_laeq_dba < 45.0) return 2;
    if (estimated_laeq_dba < 55.0) return 3;
    if (estimated_laeq_dba < 65.0) return 4;
    if (estimated_laeq_dba < 70.0) return 5;
    if (estimated_laeq_dba < 85.0) return 6;

    return 7;
}



void MicMetrics_ProcessFrame(const int16_t* pcm, uint32_t len, uint16_t* out_laeq_x10, uint8_t* out_env_class) {
    uint32_t count = len;
    
    AudioBasicFeatureDebug features = Audio_ComputeBasicFeatures(pcm, count);
    Audio_ComputeAWeightedFeatures(pcm, count, 48000, features.mean_counts, &features.a_weighted_rms_counts, &features.a_weighted_rms_dbfs);
    
    double estimated_laeq_dba = features.a_weighted_rms_dbfs + 122.40; // AUDIO_SPL_CALIBRATION_OFFSET_DB
    
    *out_env_class = Audio_ClassifyEnvironment(estimated_laeq_dba);
    
    if (estimated_laeq_dba < 0.0) estimated_laeq_dba = 0.0;
    if (estimated_laeq_dba > 150.0) estimated_laeq_dba = 150.0;
    
    *out_laeq_x10 = (uint16_t)(estimated_laeq_dba * 10.0);
}
