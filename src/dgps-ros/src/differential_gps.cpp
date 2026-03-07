
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
    LOG(INFO) << "[DGPS] Starting DGPS Read Thread";
    running_ = true;
    read_thread_ = std::thread(&DifferentialGPS::read, this);
}

void DifferentialGPS::stop()
{
    LOG(INFO) << "[DGPS] Stopping DGPS Read Thread";
    running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

void DifferentialGPS::setGpsCallback(std::function<void(GlobalCoord)> callback)
{
    gpsCallback = callback;
}

void DifferentialGPS::setAttitudeCallback(std::function<void(Orientation)> callback)
{
    attitudeCallback = callback;
}

void DifferentialGPS::setDiffGpsCallback(std::function<void(DiffNavSatFix)> callback)
{
    dgpsCallback = callback;
}

void DifferentialGPS::read()
{
    while (running_) {
        std::optional<std::string> rdata = serial_.read();
        if (rdata) {
            std::string data = rdata.value();
            if (data.rfind("$GNGGA", 0) == 0) {
                NMEA::GGA gga = parser_.parse<NMEA::GGA>(data);
                if (gga.latitude != 0.0 || gga.longitude != 0.0) {
                    LOG_FIRST_N(INFO, 1) << "[DGPS] Got GGA Reading with Status: " << gga.quality;
                    if (gps_cov_) { 
                        GlobalCoord gc{gga.latitude, gga.longitude, gga.altitude, *gps_cov_, gga.quality};
                        if (gpsCallback) gpsCallback(gc);
                        if (orient_) {
                            DiffNavSatFix dnsf{gc, *orient_};
                            if (dgpsCallback) dgpsCallback(dnsf);
                        }
                    }
                }
            } else if (data.rfind("$GNGST", 0) == 0) {
                NMEA::GST gst = parser_.parse<NMEA::GST>(data);
                if (gst.lat_std != 0.0 || gst.lon_std != 0.0) {
                    LOG_FIRST_N(INFO, 1) << "[DGPS] Got Good GST Reading";
                    double x = gst.lat_std*gst.lat_std;
                    double y = gst.lon_std*gst.lon_std;
                    double z = gst.alt_std*gst.alt_std;
                    gps_cov_ = std::make_unique<Vector3>(x, y, z);

                }
            } else if (data.rfind("$PQTMTAR", 0) == 0) {
                NMEA::PQTMTAR pqt = parser_.parse<NMEA::PQTMTAR>(data);
                if (pqt.yaw != 0.0) { 
                    LOG_FIRST_N(INFO, 1) << "[DGPS] Got Good PQTMTAR Reading";
                    Vector3 vec{pqt.pitch, pqt.roll, pqt.yaw};
                    Vector3 cov{pqt.pitch_acc, pqt.roll_acc, pqt.yaw_acc};
                    orient_ = std::make_unique<Orientation>(vec, cov, pqt.quality, pqt.length);
                    if (attitudeCallback) attitudeCallback(*orient_);
                }
            }
        } 
    }
}

void DifferentialGPS::write(const std::vector<uint8_t>& data)
{
    serial_.write(data);
}
