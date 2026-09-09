#include "ips.h"
#include "car.h"

void ips_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();
    ips200_show_string(0, 0,  "Speed Demo");
    ips200_show_string(0, 20, "L:");
    ips200_show_string(0, 40, "R:");
}

void ips_show_speed(void)
{
    ips200_show_int(20, 20, car_get_speed_l(), 5);
    ips200_show_int(20, 40, car_get_speed_r(), 5);
}
