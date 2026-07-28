/*!
* @Date 2026
*
* @About Implementation of the Septentrio mosaic-G5 driver.
* Field layouts per mosaic-G5 Firmware v1.0.0 Reference Guide, Appendix C.
*/
#include <cmath>
#include <sstream>
#include <filesystem>
#include <glog/logging.h>
#include <limits>

#include "dgps/septentrio.hpp"

using namespace dgps;

// ---------- SeptentrioParser helpers ----------

std::vector<std::string> SeptentrioParser::split(const std::string& s, char delim)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) out.push_back(item);
    return out;
}

std::string SeptentrioParser::stripChecksum(const std::string& s)
{
    auto pos = s.find('*');
    if (pos != std::string::npos) return s.substr(0, pos);
    return s;
}

double SeptentrioParser::nmeaToDeg(const std::string& val, const std::string& dir)
{
    if (val.size() < 4) return 0.0;
    int deg_len = (dir == "N" || dir == "S") ? 2 : 3;
    double deg = std::stod(val.substr(0, deg_len));
    double min = std::stod(val.substr(deg_len));
    double out = deg + (min / 60.0);
    if (dir == "S" || dir == "W") out = -out;
    return out;
}

// Bounds-safe field access — trailing empty NMEA fields are dropped by split().
const std::string& SeptentrioParser::field(const std::vector<std::string>& f, size_t idx)
{
    static const std::string empty;
    return idx < f.size() ? f[idx] : empty;
}

double SeptentrioParser::safeStod(const std::string& s, double fallback)
{
    if (s.empty()) return fallback;
    try { return std::stod(s); } catch (...) { return fallback; }
}

int SeptentrioParser::safeStoi(const std::string& s, int fallback)
{
    if (s.empty()) return fallback;
    try { return std::stoi(s); } catch (...) { return fallback; }
}

// ---------- Standard NMEA ----------

SeptNMEA::GGA SeptentrioParser::parseGGA(const std::string& line)
{
    auto f = split(stripChecksum(line), ',');
    if (f.size() < 10 || f[2].empty() || f[4].empty()) return SeptNMEA::GGA();

    SeptNMEA::GGA g;
    g.timestamp  = safeStod(f[1]);
    g.latitude   = nmeaToDeg(f[2], f[3]);
    g.longitude  = nmeaToDeg(f[4], f[5]);
    g.quality    = safeStoi(f[6]);
    g.satellites = safeStoi(f[7]);
    g.hdop       = safeStod(f[8]);
    g.altitude   = safeStod(f[9]);
    g.init = true;
    return g;
}

SeptNMEA::GLL SeptentrioParser::parseGLL(const std::string& line)
{
    auto f = split(stripChecksum(line), ',');
    if (f.size() < 7 || f[1].empty() || f[3].empty()) return SeptNMEA::GLL();

    SeptNMEA::GLL g;
    g.latitude  = nmeaToDeg(f[1], f[2]);
    g.longitude = nmeaToDeg(f[3], f[4]);
    g.timestamp = safeStod(f[5]);
    g.valid = (f[6] == "A");
    g.init = true;
    return g;
}

// $--GST,utc,rms,major,minor,orient,lat_std,lon_std,alt_std*cs
SeptNMEA::GST SeptentrioParser::parseGST(const std::string& line)
{
    auto f = split(stripChecksum(line), ',');
    if (f.size() < 9) return SeptNMEA::GST();

    SeptNMEA::GST g;
    g.timestamp = safeStod(f[1]);
    g.rms       = safeStod(f[2]);
    g.major     = safeStod(f[3]);
    g.minor     = safeStod(f[4]);
    g.orient    = safeStod(f[5]);
    g.lat_std   = safeStod(f[6]);
    g.lon_std   = safeStod(f[7]);
    g.alt_std   = safeStod(f[8]);
    g.init = true;
    return g;
}

// $--HDT,heading,T*cs   — heading field is empty until the receiver has an attitude solution
SeptNMEA::HDT SeptentrioParser::parseHDT(const std::string& line)
{
    auto f = split(stripChecksum(line), ',');
    if (f.size() < 2 || f[1].empty()) return SeptNMEA::HDT();

    SeptNMEA::HDT h;
    h.heading_deg = safeStod(f[1]);
    h.init = true;
    return h;
}

// ---------- Septentrio proprietary $PSSN sentences ----------
// After split(): f[0]="$PSSN", f[1]="RBD"|"RBP"|"RBV", f[2]=UTC, f[3]=date,
// then the message payload. Layouts per Reference Guide Appendix C.1.

// $PSSN,RBD,UTC,date,azimuth,elevation,sats,quality,base_motion,corr_age,serial,base_id*cs
SeptNMEA::RBD SeptentrioParser::parseRBD(const std::string& line)
{
    auto f = split(stripChecksum(line), ',');
    if (f.size() < 8 || field(f, 4).empty()) return SeptNMEA::RBD();

    SeptNMEA::RBD r;
    r.timestamp      = safeStod(field(f, 2));
    r.azimuth_deg    = safeStod(field(f, 4));
    r.elevation_deg  = safeStod(field(f, 5));
    r.satellites     = safeStoi(field(f, 6));
    r.quality        = safeStoi(field(f, 7));
    r.base_motion    = safeStoi(field(f, 8));
    r.correction_age = safeStod(field(f, 9));
    r.init = true;
    return r;
}

// $PSSN,RBP,UTC,date,north,east,up,sats,quality,base_motion,corr_age,serial,base_id*cs
SeptNMEA::RBP SeptentrioParser::parseRBP(const std::string& line)
{
    auto f = split(stripChecksum(line), ',');
    if (f.size() < 9 || field(f, 4).empty() || field(f, 5).empty()) return SeptNMEA::RBP();

    SeptNMEA::RBP r;
    r.timestamp      = safeStod(field(f, 2));
    r.north          = safeStod(field(f, 4));
    r.east           = safeStod(field(f, 5));
    r.up             = safeStod(field(f, 6));
    r.satellites     = safeStoi(field(f, 7));
    r.quality        = safeStoi(field(f, 8));
    r.base_motion    = safeStoi(field(f, 9));
    r.correction_age = safeStod(field(f, 10));
    r.init = true;
    return r;
}

// $PSSN,RBV,UTC,date,vnorth,veast,vup,sats,quality,base_motion,corr_age,serial,base_id*cs
SeptNMEA::RBV SeptentrioParser::parseRBV(const std::string& line)
{
    auto f = split(stripChecksum(line), ',');
    if (f.size() < 9 || field(f, 4).empty() || field(f, 5).empty()) return SeptNMEA::RBV();

    SeptNMEA::RBV r;
    r.timestamp      = safeStod(field(f, 2));
    r.vel_north      = safeStod(field(f, 4));
    r.vel_east       = safeStod(field(f, 5));
    r.vel_up         = safeStod(field(f, 6));
    r.satellites     = safeStoi(field(f, 7));
    r.quality        = safeStoi(field(f, 8));
    r.base_motion    = safeStoi(field(f, 9));
    r.correction_age = safeStod(field(f, 10));
    r.init = true;
    return r;
}

// ---------- SeptentrioGPS ----------

SeptentrioGPS::SeptentrioGPS(const std::string& nmea_dev, int nmea_baud,
                             const std::string& rtcm_dev, int rtcm_baud)
    : nmea_serial_(nmea_dev, nmea_baud)
{
    if (!google::IsGoogleLoggingInitialized())
    {
        google::InitGoogleLogging("dgps");
        std::filesystem::create_directories("/tmp/dgps");
        FLAGS_log_dir = "/tmp/dgps";
        FLAGS_alsologtostderr = 1;
    }

    if (!rtcm_dev.empty())
    {
        LOG(INFO) << "[SEPT] Opening RTCM output port " << rtcm_dev << " @ " << rtcm_baud;
        rtcm_serial_ = std::make_unique<SerialCore>(rtcm_dev, rtcm_baud);
    }
    else
    {
        LOG(INFO) << "[SEPT] RTCM output disabled (no rtcm_dev set)";
    }
}

SeptentrioGPS::~SeptentrioGPS()
{
    stop();
}

void SeptentrioGPS::start()
{
    LOG(INFO) << "[SEPT] Starting Septentrio read thread";
    running_ = true;
    read_thread_ = std::thread(&SeptentrioGPS::read, this);
}

void SeptentrioGPS::stop()
{
    running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
}

void SeptentrioGPS::setGpsCallback(std::function<void(GlobalCoord)> cb)       { gpsCallback_ = std::move(cb); }
void SeptentrioGPS::setAttitudeCallback(std::function<void(Orientation)> cb)  { attitudeCallback_ = std::move(cb); }
void SeptentrioGPS::setBaselineCallback(std::function<void(Baseline)> cb)     { baselineCallback_ = std::move(cb); }
void SeptentrioGPS::setVelocityCallback(std::function<void(Velocity)> cb)     { velocityCallback_ = std::move(cb); }
void SeptentrioGPS::setDiffGpsCallback(std::function<void(DiffNavSatFix)> cb) { dgpsCallback_ = std::move(cb); }

void SeptentrioGPS::write(const std::vector<uint8_t>& data)
{
    if (rtcm_serial_)
    {
        rtcm_serial_->write(data);
        LOG_EVERY_N(INFO, 10) << "[SEPT] Forwarded " << data.size() << " RTCM bytes to port";
    }
    else
    {
        LOG_EVERY_N(WARNING, 20) << "[SEPT] RTCM received but rtcm_dev not set; dropping " << data.size() << " bytes";
    }
}

void SeptentrioGPS::read()
{
    while (running_)
    {
        auto rdata = nmea_serial_.read();
        if (!rdata) continue;
        const std::string& line = *rdata;
        if (line.size() < 6 || line[0] != '$') continue;  // guards compare(3,3,...)

        // GGA → primary fix (any talker: $GP/$GN/...)
        if (line.compare(3, 3, "GGA") == 0)
        {
            auto gga = SeptentrioParser::parseGGA(line);
            if (!gga.init || (gga.latitude == 0.0 && gga.longitude == 0.0))
            {
                LOG_FIRST_N(INFO, 1) << "[SEPT] Waiting for GGA fix";
                continue;
            }
            LOG_FIRST_N(INFO, 1) << "[SEPT] Got GGA with quality " << gga.quality;

            Vector3 cov = gps_cov_ ? *gps_cov_ : Vector3{0.0, 0.0, 0.0};
            GlobalCoord gc{gga.latitude, gga.longitude, gga.altitude, cov, gga.quality};
            if (gpsCallback_)
                gpsCallback_(gc);

            if (dgpsCallback_)
            {
                DiffNavSatFix d;
                d.gps = gc;

                if (orient_)
                {
                    d.orientation = *orient_;
                }
                else
                {
                    const double nan = std::numeric_limits<double>::quiet_NaN();

                    d.orientation.pry = Vector3{nan, nan, nan};
                    d.orientation.cov = Vector3{nan, nan, nan};
                    d.orientation.status = 0;
                    d.orientation.baseline = nan;
                }

                dgpsCallback_(d);
            }
        }
        else if (line.compare(3, 3, "GST") == 0)
        {
            auto gst = SeptentrioParser::parseGST(line);
            if (!gst.init) continue;
            LOG_FIRST_N(INFO, 1) << "[SEPT] Got GST covariance";
            gps_cov_ = std::make_unique<Vector3>(gst.lat_std * gst.lat_std,
                                                 gst.lon_std * gst.lon_std,
                                                 gst.alt_std * gst.alt_std);
        }
        else if (line.compare(3, 3, "HDT") == 0)
        {
            auto hdt = SeptentrioParser::parseHDT(line);

            // No valid heading from the receiver.
            if (!hdt.init)
            {
                if (orient_)
                {
                    orient_->pry.z = std::numeric_limits<double>::quiet_NaN();
                    if (attitudeCallback_)
                        attitudeCallback_(*orient_);
                }
                continue;
            }

            LOG_FIRST_N(INFO, 1) << "[SEPT] Got HDT heading";

            double yaw = hdt.heading_deg * M_PI / 180.0;
            double pitch = (orient_ ? orient_->pry.x : 0.0);

            Vector3 pry{pitch, 0.0, yaw};
            orient_ = std::make_unique<Orientation>(
                pry,
                Vector3{0.0, 0.0, 0.0},
                0,
                0.0);

            if (attitudeCallback_)
                attitudeCallback_(*orient_);
        }
        else if (line.rfind("$PSSN,RBD", 0) == 0)
        {
            auto rbd = SeptentrioParser::parseRBD(line);
            if (!rbd.init || rbd.quality == 0) continue;
            LOG_FIRST_N(INFO, 1) << "[SEPT] Got RBD direction (quality " << rbd.quality << ")";

            // HDT is the single source of heading (yaw); RBD only contributes
            // pitch (elevation). Keep yaw from the last HDT solution and do NOT
            // fire the attitude callback here, otherwise /sept/heading would
            // alternate between the HDT and RBD values each cycle.
            double yaw   = (orient_ ? orient_->pry.z : 0.0);
            double pitch = rbd.elevation_deg * M_PI / 180.0;
            Vector3 pry{pitch, 0.0, yaw};
            orient_ = std::make_unique<Orientation>(pry, Vector3{0.0, 0.0, 0.0}, rbd.quality, 0.0);
        }
        else if (line.rfind("$PSSN,RBP", 0) == 0)
        {
            auto rbp = SeptentrioParser::parseRBP(line);
            if (!rbp.init || rbp.quality == 0) continue;
            LOG_FIRST_N(INFO, 1) << "[SEPT] Got RBP position (quality " << rbp.quality << ")";

            Baseline b;
            b.delta   = Vector3{rbp.east, rbp.north, rbp.up};  // ENU
            b.length  = std::sqrt(rbp.east * rbp.east + rbp.north * rbp.north + rbp.up * rbp.up);
            b.azimuth = std::atan2(rbp.east, rbp.north);        // NED heading of baseline
            b.quality = rbp.quality;
            b.init = true;
            if (baselineCallback_) baselineCallback_(b);
        }
        else if (line.rfind("$PSSN,RBV", 0) == 0)
        {
            auto rbv = SeptentrioParser::parseRBV(line);
            if (!rbv.init || rbv.quality == 0) continue;
            LOG_FIRST_N(INFO, 1) << "[SEPT] Got RBV velocity (quality " << rbv.quality << ")";

            Velocity v;
            v.v       = Vector3{rbv.vel_east, rbv.vel_north, rbv.vel_up};  // ENU
            v.quality = rbv.quality;
            if (velocityCallback_) velocityCallback_(v);
        }
        else if (line.compare(3, 3, "GLL") == 0)
        {
            // Parsed for completeness; redundant with GGA so not published.
            (void)SeptentrioParser::parseGLL(line);
        }
    }
}
