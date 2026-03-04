/*! 
* @Author Ethan Sanchez, Jason Hughes
* @Date March 2026
*
* @About Handle the serial connection to the dgps device
*/

#include <string>
#include <vector>
#include <utility>
#include <optional>
#include <iostream>
#include <sstream>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

namespace dgps
{
namespace NMEA 
{
struct GGA
{
    GGA() = default;
    GGA(double t, double lat, double lon, double a, double q, double s, double h)
    {
        timestamp = t;
        latitude = lat;
        longitude = lon;
        altitude = a;
        quality = q;
        satellites = s;
        hdop = h;
        init = true;
    }

    double timestamp;
    double latitude;
    double longitude;
    double altitude;
    int quality;
    int satellites;
    double hdop;
    bool init{false};
};

struct GST
{
    GST() = default;
    GST(double t, double r, double ma, double mi, double o, double lts, double lns, double as)
    {
        timestamp = t;
        rms = r;
        major = ma;
        minor = mi;
        orient = o;
        lat_std = lts;
        lon_std = lns;
        alt_std = as;
        init = true;
    }

    double timestamp;
    double rms;
    double major;
    double minor;
    double orient;
    double lat_std;
    double lon_std;
    double alt_std;
    bool init{false};
};

struct PQTMTAR
{
    PQTMTAR() = default;
    PQTMTAR(double t, int q, double l, double p, double r, double y, double pa, double ra, double ya, double s)
    {
        timestamp = t;
        quality = q;
        length = l;
        pitch = p;
        roll = r;
        yaw = y;
        pitch_acc = pa;
        roll_acc = ra;
        yaw_acc = ya;
        satellites = s;
        init = true;
    }
    double timestamp;
    int quality;
    double length;
    double pitch;
    double roll;
    double yaw;
    double pitch_acc;
    double roll_acc;
    double yaw_acc;
    int satellites;
    bool init{false};
};
} // namespace NMEA


class SerialCore
{
    public:
        SerialCore() = default;
        SerialCore(const std::string& dev, int baud);

        std::optional<std::string> read();
        void write();
    private:
        LibSerial::SerialPort serial_;
};

class SerialParser
{
    public:
        SerialParser() = default;

        template <typename T>
        T parse(const std::string& nmea);

    private:

        NMEA::GGA parseGGA(const std::string& nmea);
        NMEA::GST parseGST(const std::string& nmea);
        NMEA::PQTMTAR parsePQTMTAR(const std::string& nmea);

        static std::vector<std::string> splitString(const std::string& s, char delim);
        static std::string stripChecksum(const std::string& nmea);
        static double nmeaToDeg(const std::string& val, const std::string& dir);
};
}
