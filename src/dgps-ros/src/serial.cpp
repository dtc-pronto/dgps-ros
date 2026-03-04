

#include "dgps/serial.hpp"
#include <random>

using namespace dgps;

SerialCore::SerialCore(const std::string& dev, int baud)
{
    std::cout << "[DGPS] [SERIAL] Opening " << dev << " at " << baud << std::endl;
    serial_.Open(dev);
    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_460800);
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    std::cout << "[DGPS] [SERIAL] Connected to GPS" << std::endl;
}

std::optional<std::string> SerialCore::read()
{
    if (!serial_.IsOpen()) {
        throw std::runtime_error("Trying to read from serial port before its open");
        return std::nullopt;
    }

    std::string line;
    try {
        serial_.ReadLine(line, '\n', 1000); // Timeout = 1 sec
    } catch (const LibSerial::ReadTimeout &) {
        std::cout << "[DGPS] [SERIAL] Serial Read Timeout" << std::endl;
        return std::nullopt;
    } catch (const std::exception &e) {
        std::cout << "[DGPS] [SERIAL] Error: " << e.what() << std::endl;
        return std::nullopt;
    }

    if (line.empty()) return std::nullopt;

    if (line[0] != '$') {
        return std::nullopt;
    }

    return line;
}

void SerialCore::write(const std::vector<uint8_t>& data)
{
    LibSerial::DataBuffer buf(data.begin(), data.end());
    serial_.Write(buf);
}

template <typename T>
T SerialParser::parse(const std::string& nmea)
{
    if constexpr (std::is_same_v<T, NMEA::GGA>) {
        return parseGGA(nmea);
    } else if constexpr (std::is_same_v<T, NMEA::GST>) {
        return parseGST(nmea);
    } else if constexpr (std::is_same_v<T, NMEA::PQTMTAR>) {
        return parsePQTMTAR(nmea);
    } else {
        std::cout << "[DGPS] [PARSER] Called parser with unsupported type" << std::endl;
    };
}

std::vector<std::string> SerialParser::splitString(const std::string &s, char delim)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        out.push_back(item);
    }
    return out;
}

std::string SerialParser::stripChecksum(const std::string& nmea)
{
    auto asterisk = nmea.find('*');
    if (asterisk != std::string::npos)
        return nmea.substr(0, asterisk);
    return nmea;
}

double SerialParser::nmeaToDeg(const std::string &val, const std::string &dir)
{
    if (val.size() < 6) return 0.0;

    int deg_len = (dir == "N" || dir == "S") ? 2 : 3;
    double deg = std::stod(val.substr(0, deg_len));
    double min = std::stod(val.substr(deg_len));

    double out = deg + (min / 60.0);
    if (dir == "S" || dir == "W") out *= -1.0;
    return out;
}


NMEA::GGA SerialParser::parseGGA(const std::string& nmea)
{
    std::vector<std::string> f = splitString(stripChecksum(nmea), ',');
    if (f[1].empty() || f[2].empty() || f[4].empty()) return NMEA::GGA();
    try {
        double lat = nmeaToDeg(f[2], f[3]);
        double lon = nmeaToDeg(f[4], f[5]);
        
        double ts = std::stod(f[1]);

        double alt = 0.0;
        if (!f[9].empty()) alt = std::stod(f[9]);

        int fix_quality = 0;
        if (!f[6].empty()) fix_quality = std::stoi(f[6]);

        double hdop = 0.0;
        if (!f[8].empty()) hdop = std::stod(f[8]);

        int sats = 0;
        if (!f[7].empty()) sats = std::stoi(f[7]);

        NMEA::GGA gga(ts, lat, lon, alt, fix_quality, sats, hdop);
        return gga;
    } catch (const std::invalid_argument& e) {
        std::cerr << "[DGPS] [SERIAL] Invalid Arg parsing GGA : " << e.what() << " for string " << nmea << std::endl; 
    }
    return NMEA::GGA();
}

NMEA::GST SerialParser::parseGST(const std::string& nmea)
{
    std::vector<std::string> f = splitString(stripChecksum(nmea), ',');
    if (f.size() < 9) return NMEA::GST();

    try {
        double ts = 0.0;
        if (!f[1].empty()) ts = std::stod(f[1]);

        double rms = 0.0;
        if (!f[2].empty()) rms = std::stod(f[2]);

        double major = 0.0;
        if (!f[3].empty()) major = std::stod(f[3]);

        double minor = 0.0;
        if (!f[4].empty()) minor = std::stod(f[4]);

        double orient = 0.0;
        if (!f[5].empty()) orient = std::stod(f[5]);

        double lat_std = 0.0;
        if (!f[6].empty()) lat_std = std::stod(f[6]);

        double lon_std = 0.0;
        if (!f[7].empty()) lon_std = std::stod(f[7]);

        double alt_std = 0.0;
        if (!f[8].empty()) alt_std = std::stod(f[8]);
    
        NMEA::GST gst(ts, rms, major, minor, orient, lat_std, lon_std, alt_std);
        return gst;
    } catch (const std::invalid_argument& e) {
        std::cerr << "[DGPS] [SERIAL] Invalid Arg parsing GST : " << e.what() << " for string " << nmea << std::endl; 
    }

    return NMEA::GST();
}

NMEA::PQTMTAR SerialParser::parsePQTMTAR(const std::string& nmea)
{
    std::vector<std::string> f = splitString(stripChecksum(nmea), ',');
    if (f.size() < 11) return NMEA::PQTMTAR();

    double ts = 0.0;
    if (!f[2].empty()) ts = std::stod(f[2]);

    int quality = 0;
    if (!f[3].empty()) quality = std::stoi(f[3]);

    // f[4] is always null

    double length = 0.0;
    if (!f[5].empty()) length = std::stod(f[5]);

    double pitch = 0.0;
    if (!f[6].empty()) pitch = std::stod(f[6]);

    double roll = 0.0;
    if (!f[7].empty()) roll = std::stod(f[7]);

    double yaw = 0.0;
    if (!f[8].empty()) yaw = std::stod(f[8]);

    double pitch_acc = 0.0;
    if (!f[9].empty()) pitch_acc = std::stod(f[9]);

    double roll_acc = 0.0;
    if (!f[10].empty()) roll_acc = std::stod(f[10]);

    double yaw_acc = 0.0;
    if (!f[11].empty()) yaw_acc = std::stod(f[11]);

    int sats = 0;
    if (!f[12].empty()) sats = std::stod(f[12]);

    NMEA::PQTMTAR pqtmtar(ts, quality, length, pitch, roll, yaw, pitch_acc, roll_acc, yaw_acc, sats);
    return pqtmtar;
}

template NMEA::GGA SerialParser::parse<NMEA::GGA>(const std::string& nmea);
template NMEA::GST SerialParser::parse<NMEA::GST>(const std::string& nmea);
template NMEA::PQTMTAR SerialParser::parse<NMEA::PQTMTAR>(const std::string& nmea);
