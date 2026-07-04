#include "light_temperature.h"
#include <math.h>
#include <stdint.h>
#include <sys/_intsup.h>
#include <sys/types.h>


void get_relative_light_colors(uint16_t * ch, float * rel_ch)
{
    uint16_t total_light;
    
    total_light = ch[0]+ch[1]+ch[2]+ch[3]+ch[4]+ch[5]+ch[6]+ch[7];

    for (uint16_t i=0; i<CHANNEL_COUNT; i++) {
        rel_ch[i] = ch[i]/total_light;
    }
}

void get_RGB(float * rel_ch, int * RGB_total)
{
    float RGB_ch[CHANNEL_COUNT][RGB_COUNT];
    int RGB_colors[CHANNEL_COUNT][RGB_COUNT];

    int purple[] = PURPLE;
    int blue[] = BLUE;
    int light_blue[] = LIGHT_BLUE;
    int green[] = GREEN;
    int light_green[] = LIGHT_GREEN;
    int yellow[] = YELLOW;
    int orange[] = ORANGE;
    int red[] = RED;


    for (uint i=0; i<RGB_COUNT; i++)
    {
        RGB_colors[0][i] = purple[i];
        RGB_colors[1][i] = blue[i];
        RGB_colors[2][i] = light_blue[i];
        RGB_colors[3][i] = green[i];
        RGB_colors[4][i] = light_green[i];
        RGB_colors[5][i] = yellow[i];
        RGB_colors[6][i] = orange[i];
        RGB_colors[7][i] = red[i];
    
    }

    for (uint16_t i=0; i<CHANNEL_COUNT; i++) {
        for (uint16_t j=0; j<RGB_COUNT; j++) {
            RGB_ch[i][j] = rel_ch[i]*RGB_colors[i][j];
        }
    }

    for (uint16_t i=0; i<CHANNEL_COUNT; i++) 
    {
        for (uint16_t j=0; j<RGB_COUNT; j++) {
            RGB_total[j] += RGB_ch[i][j];
        }
    }
}

uint16_t RGB_to_temperature(int * RGB_total)
{
    uint16_t kelvin_table[][RGB_COUNT+1] = {
    {1000,   255, 56, 0},
    {1100,   255, 71, 0},
    {1200,   255, 83, 0},
    {1300,   255, 93, 0},
    {1400,   255, 101, 0},
    {1500,   255, 109, 0},
    {1600,   255, 115, 0},
    {1700,   255, 121, 0},
    {1800,   255, 126, 0},
    {1900,   255, 131, 0},
    {2000,   255, 138, 18},
    {2100,   255, 142, 33},
    {2200,   255, 147, 44},
    {2300,   255, 152, 54},
    {2400,   255, 157, 63},
    {2500,   255, 161, 72},
    {2600,   255, 165, 79},
    {2700,   255, 169, 87},
    {2800,   255, 173, 94},
    {2900,   255, 177, 101},
    {3000,   255, 180, 107},
    {3100,   255, 184, 114},
    {3200,   255, 187, 120}, 
    {3300,   255, 190, 126}, 
    {3400,   255, 193, 132}, 
    {3500,   255, 196, 137} ,
    {3600,   255, 199, 143} ,
    {3700,   255, 201, 148} ,
    {3800,   255, 204, 153} ,
    {3900,   255, 206, 159} ,
    {4000,   255, 209, 163} ,
    {4100,   255, 211, 168}, 
    {4200,   255, 213, 173}, 
    {4300,   255, 215, 177}, 
    {4400,   255, 217, 182} ,
    {4500,   255, 219, 186} ,    
    {4600,   255, 221, 190} ,
    {4700,   255, 223, 194} ,
    {4800,   255, 225, 198} ,
    {4900,   255, 227, 202} ,
    {5000,   255, 228, 206} ,
    {5100,   255, 230, 210} ,
    {5200,   255, 232, 213} ,
    {5300,   255, 233, 217} ,
    {5400,   255, 235, 220} ,
    {5500,   255, 236, 224} ,
    {5600,   255, 238, 227} ,
    {5700,   255, 239, 230} ,
    {5800,   255, 240, 233} ,
    {5900,   255, 242, 236} ,
    {6000,   255, 243, 239} ,
    {6100,   255, 244, 242} ,
    {6200,   255, 245, 245} ,
    {6300,   255, 246, 247} ,
    {6400,   255, 248, 251} ,
    {6500,   255, 249, 253} ,
    {6600,   254, 249, 255} ,
    {6700,   252, 247, 255} ,
    {6800,   249, 246, 255} ,
    {6900,   247, 245, 255} ,
    {7000,   245, 243, 255} ,
    {7100,   243, 242, 255} ,
    {7200,   240, 241, 255} ,
    {7300,   239, 240, 255} ,
    {7400,   237, 239, 255} ,
    {7500,   235, 238, 255} ,
    {7600,   233, 237, 255} ,
    {7700,   231, 236, 255} ,
    {7800,   230, 235, 255} ,
    {7900,   228, 234, 255} ,
    {8000,   227, 233, 255} ,
    {8100,   225, 232, 255},
    {8200,   224, 231, 255} ,
    {8300,   222, 230, 255} ,
    {8400,   221, 230, 255},
    {8500,   220, 229, 255},
    {8600,   218, 229, 255},
    {8700,   217, 227, 255},
    {8800,   216, 227, 255},
    {8900,   215, 226, 255},
    {9000,   214, 225, 255},
    {9100,   212, 225, 255},
    {9200,   211, 224, 255},
    {9300,   210, 223, 255},
    {9400,   209, 223, 255},
    {9500,   208, 222, 255},
    {9600,   207, 221, 255},
    {9700,   207, 221, 255},
    {9800,   206, 220, 255},
    {9900,   205, 220, 255},
    {10000,   207, 218, 255},
    {10100,   207, 218, 255},
    {10200,   206, 217, 255}, 
    {10300,   205, 217, 255}, 
    {10400,   204, 216, 255}, 
    {10500,   204, 216, 255}, 
    {10600,   203, 215, 255}, 
    {10700,   202, 215, 255}, 
    {10800,   202, 214, 255}, 
    {10900,   201, 214, 255}, 
    {11000,   200, 213, 255}, 
    {11100,   200, 213, 255}, 
    {11200,   199, 212, 255}, 
    {11300,   198, 212, 255}, 
    {11400,   198, 212, 255}, 
    {11500,   197, 211, 255},
    {11600,   197, 211, 255}, 
    {11700,   197, 210, 255} ,
    {11800,   196, 210, 255} ,
    {11900,   195, 210, 255} ,
    {12000,   195, 209, 255}
    };

    uint16_t tol_RB = 60;
    uint16_t tol_G = 100;
    int rel_RGB_total[RGB_COUNT];
    int temp;

    if (RGB_total[0] > RGB_total[1] && RGB_total[0] > RGB_total[2])
    {
        rel_RGB_total[0] = 255;
        rel_RGB_total[1] = (uint16_t)(RGB_total[1]/RGB_total[0]);
        rel_RGB_total[2] = (uint16_t)(RGB_total[2]/RGB_total[0]);
    }
    else if (RGB_total[2] > RGB_total[0] && RGB_total[2] > RGB_total[1])
    {
        rel_RGB_total[0] = (uint16_t)(RGB_total[0]/RGB_total[2]);
        rel_RGB_total[1] = (uint16_t)(RGB_total[1]/RGB_total[2]);
        rel_RGB_total[2] = 255;
    }
    if (rel_RGB_total[0] == 255)
    {
        for (uint16_t i=0; i<57; i++)
        {
            if (sqrt(pow(kelvin_table[i][2] - rel_RGB_total[1], 2))<tol_G && sqrt(pow(kelvin_table[i][3] - rel_RGB_total[2], 2))<tol_RB)
            {
                temp = kelvin_table[i][0];
            }
        }
    }
    else if (rel_RGB_total[2] == 255)
    {
        for (uint16_t i=57; i<111; i++)
        {
            if (sqrt(pow(kelvin_table[i][2] - rel_RGB_total[1], 2))<tol_G && sqrt(pow(kelvin_table[i][1] - rel_RGB_total[0], 2))<tol_RB)
            {
                temp = kelvin_table[i][0];
            }
        }
    }
    return temp;
}

void remove_outsiders(int * temps, int * cleaned_temps)
{
    int current_temps[3];
    float current_distances[3]; // 12, 13, 23

    cleaned_temps[0] = temps[0];
    cleaned_temps[SAMPLE_SIZE-1] = temps[SAMPLE_SIZE-1];

    for (uint16_t i=0; i<SAMPLE_SIZE-2; i++) {
        current_temps[0] = temps[i];
        current_temps[1] = temps[i+1];
        current_temps[2] = temps[i+2];

        current_distances[0] = sqrt(pow(current_temps[0]-current_temps[1], 2));
        current_distances[1] = sqrt(pow(current_temps[0]-current_temps[2], 2));
        current_distances[2] = sqrt(pow(current_temps[1]-current_temps[2], 2));

        if (current_distances[1] < current_distances[0] && current_distances[1] < current_distances[2])
        {
            if (current_distances[0] > current_distances[2])
            {
                cleaned_temps[i+1] = temps[i+2];
            }
            else
            {
                cleaned_temps[i+1] = temps[i];
            }
        }
    }
}

int get_light_temperature(uint16_t * samples)
{
    float rel_samples[CHANNEL_COUNT];
    int RGB[RGB_COUNT];
    uint16_t temp;
    
    get_relative_light_colors(samples, rel_samples);    
    get_RGB(rel_samples, RGB);
    temp = RGB_to_temperature(RGB);

    return temp;
}

int get_avg_light_temperature(int * temps)
{
    int cleaned_temps[SAMPLE_SIZE];
    int average_temp=0;

    remove_outsiders(temps, cleaned_temps);
    for (uint16_t i=0; i<SAMPLE_SIZE; i++) {
        average_temp += cleaned_temps[i];
        average_temp = average_temp/SAMPLE_SIZE;
    }
    return average_temp;
}