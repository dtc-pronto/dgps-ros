/*!
* @Author Jason Hughes
* @Date March 2026
*
* @About Wrap the serial core and parser into a 
* nice api
*/

#include <atomic>
#include <thread>
#include <functional>
#include "dgps/serial.hpp"

namespace dgps
{
struct Vector3
{
    double x{-1}, y{-1}, z{-1};
};
struct GlobalCoord
{
    double latitude;
    double longitude;
    double altitude;
};
struct NavSatFix
{
    GlobalCoord gps;
    Vector3 variance;
    int status;
};


class DifferentialGPS
{
    public:
        DifferentialGPS() = default;
        DifferentialGPS(const std::string& dev, int baud);
        ~DifferentialGPS();

        void start();
        void stop();

        void setGpsCallback(std::function<void(NavSatFix)> callback);
        void setAttitudeCallback(std::function<void(Vector3)> callback);

        void writeRtcm();

    private:
        void read();
        
        SerialCore serial_;
        SerialParser parser_;

        std::thread read_thread_;
        std::atomic<bool> running_{false};

        std::function<void(NavSatFix)> gpsCallback;
        std::function<void(Vector3)> attitudeCallback;

        Vector3 cov_;
};
}
