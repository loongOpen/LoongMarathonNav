//
// Created by remotelink on 25-6-3.
//

#ifndef TYPE_H
#define TYPE_H

// #include <Eigen/Core>
//
// struct NEMA_MSG{
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// double time;
// Eigen::Vector3d position;
// Eigen::Vector3d pos_accuracy;
// Eigen::Vector3d velocity;
// Eigen::Vector3d std_velocity;
// Eigen::Vector3d pose;
// Eigen::Vector3d std_pose;
// double rtk_state;
//
// NEMA_MSG(){
//     time = 0;
//     position = Eigen::Vector3d::Zero();
//     pos_accuracy = Eigen::Vector3d::Zero();
//     velocity = Eigen::Vector3d::Zero();
//     std_velocity = Eigen::Vector3d::Zero();
//     pose = Eigen::Vector3d::Zero();
//     std_pose = Eigen::Vector3d::Zero();
//     rtk_state = 0;
// }
//
// NEMA_MSG &operator = (const NEMA_MSG &other)
// {
//     time = other.time;
//     position = other.position;
//     pos_accuracy = other.pos_accuracy;
//     velocity = other.velocity;
//     std_velocity = other.std_velocity;
//     pose = other.pose;
//     std_pose = other.std_pose;
//     rtk_state = other.rtk_state;
//     return *this;
// }
// };

struct GPRMC {
    float utc;
    bool status;
    double lat;
    double lon;
    double speed;
    double direction;
    int day;
    int month;
    int year;
    GPRMC() {
        utc = 0;
        status = false;
        lat = 0;
        lon = 0;
        speed = 0;
        direction = 0;
        day = 0;
        month = 0;
        year = 0;
    }
};

struct KSXT_UTC {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t microsecond;
    KSXT_UTC() {
        year = 0;
        month = 0;
        day = 0;
        hour = 0;
        minute = 0;
        second = 0;
        microsecond = 0;
    }
};

struct KSXT1 {
    double utc;
    double lat;
    double lon;
    double height;
    double vel;
    double heading;
    double pitch;
    double roll;
    int pos_qual;
    int heading_qual;
};
struct KSXT_DATA {
    double lat;
    double lon;
    double height;
    double heading;
    double pitch;
    double roll;
    double east;
    double north;
    double up;
    double vel;
    double vel_east;
    double vel_north;
    double vel_up;
    unsigned int pos_qual;
    unsigned int heading_qual;
    KSXT_DATA() {
        lat = 0.0;
        lon = 0.0;
        height = 0.0;
        heading = 0.0;
        pitch = 0.0;
        roll = 0.0;
        east = 0.0;
        north = 0.0;
        up = 0.0;
        vel = 0.0;
        vel_east = 0.0;
        vel_north = 0.0;
        vel_up = 0.0;
        pos_qual = 0;
        heading_qual = 0;

    }
};
struct KSXT {
    KSXT_UTC utc;
    KSXT_DATA data;
    KSXT() {
        utc = KSXT_UTC();
        data = KSXT_DATA();
    }
};



/*
# A more complete GPS fix to supplement sensor_msgs/NavSatFix.
std_msgs/Header header

GPSStatus status

# Latitude (degrees). Positive is north of equator; negative is south.
float64 latitude

# Longitude (degrees). Positive is east of prime meridian, negative west.
float64 longitude

# Altitude (meters). Positive is above reference (e.g., sea level).
float64 altitude

# Direction (degrees from north)
float64 track

# Ground speed (meters/second)
float64 speed

# Vertical speed (meters/second)
float64 climb

# Device orientation (units in degrees)
float64 pitch
float64 roll
float64 dip

# GPS time
float64 time

## Dilution of precision; Xdop<=0 means the value is unknown

# Total (positional-temporal) dilution of precision
float64 gdop

# Positional (3D) dilution of precision
float64 pdop

# Horizontal dilution of precision
float64 hdop

# Vertical dilution of precision
float64 vdop

# Temporal dilution of precision
float64 tdop

## Uncertainty of measurement, 95% confidence

# Spherical position uncertainty (meters) [epe]
float64 err

# Horizontal position uncertainty (meters) [eph]
float64 err_horz

# Vertical position uncertainty (meters) [epv]
float64 err_vert

# Track uncertainty (degrees) [epd]
float64 err_track

# Ground speed uncertainty (meters/second) [eps]
float64 err_speed

# Vertical speed uncertainty (meters/second) [epc]
float64 err_climb

# Temporal uncertainty [ept]
float64 err_time

# Orientation uncertainty (degrees)
float64 err_pitch
float64 err_roll
float64 err_dip

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.

float64[9] position_covariance

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type
*/

#endif //TYPE_H
