#include "as7341_driver.h"

#define SAMPLE_SIZE 10
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

void get_relative_light_colors(uint16_t * ch, float * rel_ch);
void get_RGB(float * rel_ch, int * RGB_total);
uint16_t RGB_to_temperature(int * RGB_total);
void remove_outsiders(int * temps, int * cleaned_temps);
int get_light_temperature(uint16_t * samples);
int get_avg_light_temperature(int * temps);