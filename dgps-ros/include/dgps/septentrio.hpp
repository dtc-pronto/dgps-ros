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
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "dgps/serial.hpp"

namespace dgps
{

// -----------------------------------------------------------------------------
// Data types used by the Septentrio driver
// -----------------------------------------------------------------------------

struct Vector3
{
    Vector3() = default;
    Vector3(double x, double y, double z) : x{x}, y{y}, z{z} {}

    double x{-1.0};
    double y{-1.0};
    double z{-1.0};
};

struct GlobalCoord
{
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
    Vector3 covariance;
    int status{0};
};

struct Orientation
{
    Orientation() = default;

    Orientation(Vector3 o, Vector3 c, int s, double b)
        : pry{o}, cov{c}, status{s}, baseline{b} {}

    Vector3 pry;
    Vector3 cov;
    int status{0};
    double baseline{0.0};
};

struct Baseline
{
    double north{0.0};
    double east{0.0};
    double down{0.0};
    double length{0.0};
    int status{0};
};

struct DiffNavSatFix
{
    GlobalCoord gps;
    Orientation orientation;
};

// -----------------------------------------------------------------------------
// Septentrio NMEA structures
// -----------------------------------------------------------------------------

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

// $--GST pseudorange error statistics
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
    double heading_deg{0.0};
    bool init{false};
};

// $PSSN,RBD — Rover-Base Direction
struct RBD
{
    double timestamp{0.0};
    double azimuth_deg{0.0};
    double elevation_deg{0.0};
    int satellites{0};
    int quality{0};
    int base_motion{0};
    double correction_age{0.0};
    bool init{false};
};

// $PSSN,RBP — Rover-Base Position
struct RBP
{
    double timestamp{0.0};
    double north{0.0};
    double east{0.0};
    double up{0.0};
    int satellites{0};
    int quality{0};
    int base_motion{0};
    double correction_age{0.0};
    bool init{false};
};

// $PSSN,RBV — Rover-Base Velocity
struct RBV
{
    double timestamp{0.0};
    double vel_north{0.0};
    double vel_east{0.0};
    double vel_up{0.0};
    int satellites{0};
    int quality{0};
    int base_motion{0};
    double correction_age{0.0};
    bool init{false};
};

} // namespace SeptNMEA

// -----------------------------------------------------------------------------
// Septentrio parser
// -----------------------------------------------------------------------------

class SeptentrioParser
{
public:
    static SeptNMEA::GGA parseGGA(const std::string& line);
    static SeptNMEA::GLL parseGLL(const std::string& line);
    static SeptNMEA::GST parseGST(const std::string& line);
    static SeptNMEA::HDT parseHDT(const std::string& line);
    static SeptNMEA::RBD parseRBD(const std::string& line);
    static SeptNMEA::RBP parseRBP(const std::string& line);
    static SeptNMEA::RBV parseRBV(const std::string& line);

private:
    static std::vector<std::string> split(
        const std::string& s, char delim);

    static std::string stripChecksum(
        const std::string& s);

    static double nmeaToDeg(
        const std::string& val,
        const std::string& dir);

    static const std::string& field(
        const std::vector<std::string>& f,
        size_t idx);

    static double safeStod(
        const std::string& s,
        double fallback = 0.0);

    static int safeStoi(
        const std::string& s,
        int fallback = 0);
};

// -----------------------------------------------------------------------------
// Septentrio GPS driver
// -----------------------------------------------------------------------------

class SeptentrioGPS
{
public:
    SeptentrioGPS() = default;

    SeptentrioGPS(
        const std::string& nmea_dev,
        int nmea_baud,
        const std::string& rtcm_dev,
        int rtcm_baud);

    ~SeptentrioGPS();

    void start();
    void stop();

    void setGpsCallback(
        std::function<void(GlobalCoord)> cb);

    void setAttitudeCallback(
        std::function<void(Orientation)> cb);

    void setBaselineCallback(
        std::function<void(Baseline)> cb);

    void setDiffGpsCallback(
        std::function<void(DiffNavSatFix)> cb);

    void write(const std::vector<uint8_t>& data);

private:
    void read();

    SerialCore nmea_serial_;
    std::unique_ptr<SerialCore> rtcm_serial_;

    std::thread read_thread_;
    std::atomic<bool> running_{false};

    std::function<void(GlobalCoord)> gpsCallback_;
    std::function<void(Orientation)> attitudeCallback_;
    std::function<void(Baseline)> baselineCallback_;
    std::function<void(DiffNavSatFix)> dgpsCallback_;

    std::unique_ptr<Vector3> gps_cov_;
    std::unique_ptr<Orientation> orient_;
};

} // namespace dgps