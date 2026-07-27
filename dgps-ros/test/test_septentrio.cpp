/*!
* @Date 2026
*
* @About Offline parser checks for the Septentrio driver, run against real
* mosaic-G5 NMEA captured from the SimpleRTK 4 Heading. No hardware needed.
*/
#include <cmath>
#include <iostream>
#include <string>

#include "dgps/septentrio.hpp"

using namespace dgps;

static int failures = 0;

static void check(bool cond, const std::string& what)
{
    std::cout << (cond ? "[PASS] " : "[FAIL] ") << what << "\n";
    if (!cond) ++failures;
}

static bool close(double a, double b, double tol = 1e-4)
{
    return std::fabs(a - b) <= tol;
}

int main()
{
    // GGA — standard fix
    {
        auto g = SeptentrioParser::parseGGA(
            "$GNGGA,201028.00,3956.4822092,N,07511.9727400,W,1,13,1.0,1.2844,M,-33.9408,M,,*4A");
        check(g.init, "GGA parses");
        check(close(g.latitude, 39.9413702), "GGA latitude");
        check(close(g.longitude, -75.1995457), "GGA longitude");
        check(g.quality == 1, "GGA quality");
        check(g.satellites == 13, "GGA satellites");
        check(close(g.altitude, 1.2844), "GGA altitude");
    }

    // GST — Septentrio fills only lat/lon/alt std (fields 6,7,8); 2..5 are empty
    {
        auto s = SeptentrioParser::parseGST("$GNGST,201028.00,,,,,2.059,1.927,7.196*4A");
        check(s.init, "GST parses with empty rms/major/minor/orient");
        check(close(s.lat_std, 2.059), "GST lat_std");
        check(close(s.lon_std, 1.927), "GST lon_std");
        check(close(s.alt_std, 7.196), "GST alt_std");
    }

    // HDT — empty heading (no attitude solution) must be rejected
    {
        auto h = SeptentrioParser::parseHDT("$GNHDT,,T*05");
        check(!h.init, "HDT empty heading rejected");
    }

    // HDT — populated
    {
        auto h = SeptentrioParser::parseHDT("$GNHDT,219.842,T*2F");
        check(h.init, "HDT populated parses");
        check(close(h.heading_deg, 219.842), "HDT heading");
    }

    // RBD — synthetic line in the documented field order (Ref. Guide C.1.2)
    {
        auto r = SeptentrioParser::parseRBD("$PSSN,RBD,201035.00,200526,123.4,1.2,15,4,0,2.0,SN123,4000*00");
        check(r.init, "RBD parses");
        check(close(r.azimuth_deg, 123.4), "RBD azimuth");
        check(close(r.elevation_deg, 1.2), "RBD elevation");
        check(r.satellites == 15, "RBD satellites");
        check(r.quality == 4, "RBD quality");
    }

    // RBP — North, East, Up order (Ref. Guide C.1.3)
    {
        auto r = SeptentrioParser::parseRBP("$PSSN,RBP,201035.00,200526,0.40,0.30,0.05,15,4,0,2.0,SN123,4000*00");
        check(r.init, "RBP parses");
        check(close(r.north, 0.40), "RBP north");
        check(close(r.east, 0.30), "RBP east");
        check(close(r.up, 0.05), "RBP up");
        check(r.quality == 4, "RBP quality");
    }

    std::cout << (failures == 0 ? "\nALL CHECKS PASSED\n" : "\nFAILURES: " + std::to_string(failures) + "\n");
    return failures == 0 ? 0 : 1;
}
