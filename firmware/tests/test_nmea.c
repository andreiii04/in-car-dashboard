//
// host test for the NMEA parser
//
// - nmea.c has no HAL in it, so it compiles and runs on the mac
// - the first three sentences are copied straight out of the Quectel protocol
//   spec, the rest i made up and worked out the checksum for
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/Inc tests/test_nmea.c App/Src/nmea.c -o
//  /tmp/test_nmea && /tmp/test_nmea

#include <stdio.h>
#include "nmea.h"

static int fails = 0;

static void check(const char *what, int ok) {
    printf("  %-36s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        fails++;
    }
}

// floats never compare exactly, so allow a small difference
static int near(double a, double b, double tol) {
    double d = a - b;
    if (d < 0) {
        d = -d;
    }
    return d < tol;
}

int main(void) {
    // straight out of the Quectel spec
    const char *rmc_gn = "$GNRMC,080608.000,A,3029.461489,N,11430.072002,E,0.00,148.41,210423,,,D,V*09";
    const char *rmc_gp = "$GPRMC,072316.000,A,3029.464977,N,11430.067548,E,0.00,5.09,230423,,,D,V*10";
    const char *gga_gn = "$GNGGA,072741.000,3029.463944,N,11430.070240,E,2,17,0.64,141.895,M,-13.122,M,,*5A";

    // made up, checksums worked out by hand
    const char *rmc_fast  = "$GNRMC,080608.000,A,3029.461489,N,11430.072002,E,10.00,148.41,210423,,,D,V*38";
    const char *rmc_south = "$GNRMC,080608.000,A,3029.461489,S,11430.072002,W,0.00,148.41,210423,,,D,V*06";
    const char *rmc_nofix = "$GPRMC,080608.000,V,,,,,,,210423,,,N,V*37";

    printf("talker ID - GN and GP must both be recognised\n");
    check("$GNRMC is an RMC", nmea_identify(rmc_gn) == NMEA_RMC);
    check("$GPRMC is an RMC", nmea_identify(rmc_gp) == NMEA_RMC);
    check("$GNGGA is a GGA", nmea_identify(gga_gn) == NMEA_GGA);

    printf("checksum catches damaged sentences\n");
    char broken[128];
    snprintf(broken, sizeof(broken), "%s", rmc_gn);
    broken[10] = '9';  // change one character, checksum no longer matches
    check("corrupted sentence rejected", nmea_identify(broken) == NMEA_UNKNOWN);
    check("cut off sentence rejected", nmea_identify("$GNRMC,0806") == NMEA_UNKNOWN);

    printf("RMC fields\n");
    nmea_rmc_t r = { 0 };
    check("parses", nmea_parse_rmc(rmc_gn, &r));
    check("status A means valid", r.valid);
    check("lat 30.4910248 deg", near(r.lat_deg, 30.0 + 29.461489 / 60.0, 1e-9));
    check("lon 114.5012000 deg", near(r.lon_deg, 114.0 + 30.072002 / 60.0, 1e-9));
    check("course 148.41", near(r.course_deg, 148.41, 1e-3));
    check("time 08:06:08", r.hour == 8 && r.minute == 6 && r.second == 8);
    check("date 21/04/23", r.day == 21 && r.month == 4 && r.year == 23);

    printf("knots get converted to km/h\n");
    nmea_rmc_t f = { 0 };
    check("10.00 knots is 18.52 km/h", nmea_parse_rmc(rmc_fast, &f) && near(f.speed_kmh, 18.52, 0.01));

    printf("south and west come out negative\n");
    nmea_rmc_t s = { 0 };
    check("S gives negative lat", nmea_parse_rmc(rmc_south, &s) && s.lat_deg < 0);
    check("W gives negative lon", s.lon_deg < 0);

    printf("no fix yet, but the clock still works\n");
    nmea_rmc_t n = { 0 };
    check("parses with empty fields", nmea_parse_rmc(rmc_nofix, &n));
    check("valid is false", !n.valid);
    check("time is still readable", n.time_valid && n.hour == 8);

    printf("GGA fields\n");
    nmea_gga_t g = { 0 };
    check("parses", nmea_parse_gga(gga_gn, &g));
    check("quality 2, differential", g.fix_quality == 2);
    check("17 satellites used", g.sats_used == 17);
    check("hdop 0.64", near(g.hdop, 0.64, 1e-3));
    check("altitude 141.895 m", near(g.altitude_m, 141.895, 1e-3));

    printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
    return fails != 0;
}