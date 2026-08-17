/*!
* @Date 2026
*
* @About Driver for Septentrio mosaic-G5 (SimpleRTK 4 Heading).
* Parses standard NMEA (GGA, GLL, GST, HDT) plus the Septentrio
* proprietary $PSSN sentences RBD/RBP/RBV.
*
* Field layouts follow the mosaic-G5 Firmware v1.0.0 Reference Guide,
* Appendix C (List of NMEA Sentences).
*/
#pragma once

#include <atomic>
#include <thread>
#include <functional>
#include <optional>
#include <memory>
#include <string>
#include <vector>

#include "dgps/serial.hpp"
#include "dgps/differential_gps.hpp"  // reuse Vector3, GlobalCoord, Orientation, DiffNavSatFix

namespace dgps
{
namespace SeptNMEA
{

// $--GGA standard fix data
struct GGA
{
    double timestamp{0.0};
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
    int quality{0};
    int satellites{0};
    double hdop{0.0};
    bool init{false};
};

// $--GLL geographic position
struct GLL
{
    double timestamp{0.0};
    double latitude{0.0};
    double longitude{0.0};
    bool valid{false};
    bool init{false};
};

// $--GST pseudorange error statistics — gives per-axis 1-sigma in metres
struct GST
{
    double timestamp{0.0};
    double rms{0.0};
    double major{0.0};
    double minor{0.0};
    double orient{0.0};
    double lat_std{0.0};
    double lon_std{0.0};
    double alt_std{0.0};
    bool init{false};
};

// $--HDT true heading
struct HDT
{
    double heading_deg{0.0};   // true heading [deg, 0=N, 90=E]; empty when no solution
    bool init{false};
};

// $PSSN,RBD — Rover-Base Direction  (Ref. Guide C.1.2)
// $PSSN,RBD,hhmmss.ss,ddmmyy,azimuth,elevation,sats,quality,base_motion,corr_age,serial,base_id*cs
struct RBD
{
    double timestamp{0.0};
    double azimuth_deg{0.0};    // base as seen from rover, 0..360 increasing E, deg True
    double elevation_deg{0.0};  // base as seen from rover, -90..90, deg
    int satellites{0};
    int quality{0};             // 0=Invalid 2=DGPS 4=RTK 5=Float RTK
    int base_motion{0};         // 0=static 1=moving
    double correction_age{0.0};
    bool init{false};
};

}

class SeptentrioParser
{
    public:
        static SeptNMEA::GGA parseGGA(const std::string& line);
        static SeptNMEA::GLL parseGLL(const std::string& line);
        static SeptNMEA::GST parseGST(const std::string& line);
        static SeptNMEA::HDT parseHDT(const std::string& line);
        static SeptNMEA::RBD parseRBD(const std::string& line);

    private:
        static std::vector<std::string> split(const std::string& s, char delim);
        static std::string stripChecksum(const std::string& s);
        static double nmeaToDeg(const std::string& val, const std::string& dir);
        static const std::string& field(const std::vector<std::string>& f, size_t idx);
        static double safeStod(const std::string& s, double fallback = 0.0);
        static int safeStoi(const std::string& s, int fallback = 0);
};

class SeptentrioGPS
{
    public:
        SeptentrioGPS() = default;
        SeptentrioGPS(const std::string& nmea_dev, int nmea_baud,
                      const std::string& rtcm_dev, int rtcm_baud);
        ~SeptentrioGPS();

        void start();
        void stop();

        void setGpsCallback(std::function<void(GlobalCoord)> cb);
        void setAttitudeCallback(std::function<void(Orientation)> cb);
        void setBaselineCallback(std::function<void(Baseline)> cb);
        void setDiffGpsCallback(std::function<void(DiffNavSatFix)> cb);

        void write(const std::vector<uint8_t>& data);  // RTCM out → rtcm port

    private:
        void read();

        SerialCore nmea_serial_;
        std::unique_ptr<SerialCore> rtcm_serial_;

        std::thread read_thread_;
        std::atomic<bool> running_{false};

        std::function<void(GlobalCoord)>  gpsCallback_;
        std::function<void(Orientation)>  attitudeCallback_;
        std::function<void(DiffNavSatFix)> dgpsCallback_;

        std::unique_ptr<Vector3>     gps_cov_;   // from GST [m^2]
        std::unique_ptr<Orientation> orient_;    // from HDT / RBD
};

} // namespace dgps
