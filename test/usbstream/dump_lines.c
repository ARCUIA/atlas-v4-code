/* Prints sample lines exactly as the firmware formats them, for the Python parser check. */
#include <stdio.h>
#include <string.h>
#include "usbstream.h"

int main(void)
{
    static uint8_t store[4096]; usbstream_t s; char out[2048];
    usbstream_init(&s, store, sizeof store);
    int32_t a[3] = {-86, -99, 988}, g[3] = {-280, 210, 140}, a2[3] = {16000, -15999, 0}, g2[3] = {2293000, -2293000, 5};
    int32_t a3[3] = {-1, 0, 1000}, g3[3] = {-1, 0, 1}, m[3] = {123, -45, 300};
    usbstream_imu(&s, 1234567890ull, a, g, NULL);
    usbstream_imu(&s, 1234570073ull, a2, g2, NULL);
    usbstream_imu(&s, 1234572256ull, a3, g3, NULL);
    usbstream_mag(&s, 1234567890ull, m);
    usbstream_baro(&s, 1234567890ull, 98717, 2130);
    usbstream_gps(&s, 1234567890ull, 3, 9, 416611234, -915301234, 221600, 100, 400, 700);
    usbstream_gps(&s, 1234600000ull, 0, 0, -123, 0, -500, -50, 0xFFFFFFFFu, 3750000u);
    usbstream_text(&s, "IMU:OK xl[mg]=-86,-99,988 gy[mdps]=-280,210,140 | BARO:OK 98717 Pa | SD:OK LOG:OK sess=34 | GPS:OK fix=0 sv=0 | RADIO:NO-IRQ | USB:OK ln=7 drop=0 flush=0\r\n",
                    (uint32_t)strlen("IMU:OK xl[mg]=-86,-99,988 gy[mdps]=-280,210,140 | BARO:OK 98717 Pa | SD:OK LOG:OK sess=34 | GPS:OK fix=0 sv=0 | RADIO:NO-IRQ | USB:OK ln=7 drop=0 flush=0\r\n"));
    uint32_t n = usbstream_peek(&s, (uint8_t *)out, sizeof out);
    fwrite(out, 1, n, stdout);
    return 0;
}
