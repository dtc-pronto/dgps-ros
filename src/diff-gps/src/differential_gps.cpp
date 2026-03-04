
#include <cmath>
#include "dgps/differential_gps.hpp"

using namespace dgps;

DifferentialGPS::DifferentialGPS(const std::string& dev, int baud) : serial_(dev, baud)
{
}

DifferentialGPS::~DifferentialGPS()
{
    stop();
}

void DifferentialGPS::start()
{
    running_ = true;
    read_thread_ = std::thread(&DifferentialGPS::read, this);
}

void DifferentialGPS::stop()
{
    running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

void DifferentialGPS::setGpsCallback(std::function<void(NavSatFix)> callback)
{
    gpsCallback = callback;
}

void DifferentialGPS::setAttitudeCallback(std::function<void(Vector3)> callback)
{
    attitudeCallback = callback;
}

void DifferentialGPS::read()
{
    while (running_) {
        std::optional<std::string> rdata = serial_.read();
        if (rdata) {
            std::string data = rdata.value();
            if (data.rfind("$GNGGA", 0) == 0) {
                NMEA::GGA gga = parser_.parse<NMEA::GGA>(data);
                if (cov_.x > 0) {
                    
                    GlobalCoord gc{gga.latitude, gga.longitude, gga.altitude};
                    NavSatFix nsf{gc, cov_, gga.quality};
                    if (gpsCallback) gpsCallback(nsf);
                }
            } else if (data.rfind("$GNGST", 0) == 0) {
                NMEA::GST gst = parser_.parse<NMEA::GST>(data);
                cov_.x = gst.lat_std*gst.lat_std;
                cov_.y = gst.lon_std*gst.lon_std;
                cov_.z = gst.alt_std*gst.alt_std;
            } else if (data.rfind("$PQTMTAR", 0) == 0) {
                NMEA::PQTMTAR pqt = parser_.parse<NMEA::PQTMTAR>(data);
                Vector3 vec{pqt.pitch, pqt.roll, pqt.yaw};
                if (attitudeCallback) attitudeCallback(vec);
            }
        }

        //std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}
