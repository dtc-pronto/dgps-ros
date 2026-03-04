/*!
* @Author Jason Hughes
* @Date March 2026
*
* @About test the serial connection and data output
*/

#include "dgps/differential_gps.hpp"

int main() {

    dgps::DifferentialGPS dgps("/dev/ttyACM0", 460800);
    dgps.start();

    std::this_thread::sleep_for(std::chrono::seconds(60));

    dgps.stop();

    return 0;
}
