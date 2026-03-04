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
    Vector3() = default;
    Vector3(double x, double y, double z) : x{x}, y{y}, z{z} { }
    double x{-1}, y{-1}, z{-1};
};
struct GlobalCoord
{
    double latitude;
    double longitude;
    double altitude;
    Vector3 covariance;
    int status;
};
struct Orientation
{
    Orientation() = default;
    Orientation(Vector3 o, Vector3 c, int s) : pry{o}, cov{c}, status{s} { }
    Vector3 pry;
    Vector3 cov;
    int status;
};
struct DiffNavSatFix
{
    GlobalCoord gps;
    Orientation orientation; 
};


class DifferentialGPS
{
    public:
        DifferentialGPS() = default;
        DifferentialGPS(const std::string& dev, int baud);
        ~DifferentialGPS();

        void start();
        void stop();

        void setGpsCallback(std::function<void(GlobalCoord)> callback);
        void setDiffGpsCallback(std::function<void(DiffNavSatFix)> callback);
        void setAttitudeCallback(std::function<void(Orientation)> callback);

        void write(const std::vector<uint8_t>& data);

    private:
        void read();
        
        SerialCore serial_;
        SerialParser parser_;

        std::thread read_thread_;
        std::atomic<bool> running_{false};

        std::function<void(GlobalCoord)> gpsCallback;
        std::function<void(DiffNavSatFix)> dgpsCallback;
        std::function<void(Orientation)> attitudeCallback;

        std::unique_ptr<Vector3> gps_cov_;
        std::unique_ptr<Orientation> orient_;
};
}
