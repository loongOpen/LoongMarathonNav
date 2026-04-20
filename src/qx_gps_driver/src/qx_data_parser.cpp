//
// Created by remotelink on 25-6-3.
//
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "qx_data_parser.h"

#define KMH_TO_MS(kmh) ((kmh) * 1000.0f / 3600.0f)

// 安全地将字符串转换为 double，失败时不抛出异常
bool safe_stod(const std::string& str, double& out) {
    if (str.empty()) return false;
    try {
        size_t pos = 0;
        out = std::stod(str, &pos);
        return pos == str.size();
    } catch (...) {
        return false;
    }
}

// 安全地将字符串转换为 int，失败时不抛出异常
bool safe_stoul(const std::string& str, unsigned int& out) {
    if (str.empty()) return false;
    try {
        size_t pos = 0;
        out = std::stoul(str, &pos);
        return pos == str.size();
    } catch (...) {
        return false;
    }
}

bool safe_stoi(const std::string& str, int& out) {
    try {
        size_t pos;
        out = std::stoi(str, &pos);
        return pos == str.size();
    } catch (...) {
        return false;
    }
}

// 转换并赋值宏（仅用于简化代码）
#define ASSIGN_IF_VALID(index, var, func) \
if (!tokens[(index)].empty() && func(tokens[index], var)) {}

// 字段索引常量定义（千寻 KSXT 格式）
// $KSXT,time,lon,lat,height,heading,pitch,vel_dir,vel,speed,pos_qual,heading_qual,...,vel_east,vel_north,vel_up,...
constexpr int LON_IDX = 2;
constexpr int LAT_IDX = 3;
constexpr int HEIGHT_IDX = 4;
constexpr int HEADING_IDX = 5;   // 双天线定向航向（稳定）
constexpr int PITCH_IDX = 6;
constexpr int VEL_IDX = 8;
constexpr int ROLL_IDX = 9;
constexpr int POS_QUAL_IDX = 10;
constexpr int HEADING_QUAL_IDX = 11;
constexpr int EAST_IDX = 14;
constexpr int NORTH_IDX = 15;
constexpr int UP_IDX = 16;
constexpr int VEL_EAST_IDX = 17;
constexpr int VEL_NORTH_IDX = 18;
constexpr int VEL_UP_IDX = 19;


// 计算 NMEA 格式校验和（XOR 异或）
uint8_t calculateNMEACRC(const std::string& data) {
    uint8_t crc = 0;
    for (char c : data) {
        crc ^= static_cast<uint8_t>(c);
    }
    return crc;
}

// 验证 GPRMC 消息的 CRC 校验
bool validateKSXT(const std::string& sentence) {
    // 查找 '*' 的位置
    size_t starPos = sentence.find('*');
    if (starPos == std::string::npos || starPos + 2 > sentence.size()) {
        printf("无效的消息格式（未找到 * 或校验和不足两位）\n");
        return false;
    }

    // 提取原始数据部分（从 $ 后第一个字符到 * 前）
    std::string dataPart = sentence.substr(1, starPos - 1);

    // 计算校验和
    uint8_t calculatedCRC = calculateNMEACRC(dataPart);

    // 提取接收到的校验和（十六进制）
    std::string receivedCRCHex = sentence.substr(starPos + 1, 2);
    uint8_t receivedCRC = static_cast<uint8_t>(strtol(receivedCRCHex.c_str(), nullptr, 16));

    // 对比校验和
    if (calculatedCRC == receivedCRC) {
        // printf("\n校验成功！期望: %02X，实际: %s\n", calculatedCRC, receivedCRCHex.c_str());
        return true;
    } else {
        printf("校验失败！期望: %02X，实际: %s\n %s\n", calculatedCRC, receivedCRCHex.c_str(), sentence.c_str());
        return false;
    }
}


// 构造函数
DataParser::DataParser(const std::string &dev_name, unsigned int& baud_rate) {
    serial_ptr = std::make_shared<CallbackAsyncSerial>();
    serial_name = dev_name;
    serial_baud = baud_rate;
    last_gprmc_course_ = 0.0;
    last_gprmc_valid_ = false;
}

DataParser::~DataParser() {
    serial_ptr->close();
}

// 设置回调函数（回调函数为发布函数）
void DataParser::setCallback(const boost::function<void(const KSXT* msg)> &call_back) {
    rtk_callback = call_back;
}

// 执行数据的接受、解析
bool DataParser::run_parse() {
    // 打开串口
    try {
        serial_ptr->open(serial_name, serial_baud);
    } catch (const boost::system::system_error& e) {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
        return false;
    }
    if (!serial_ptr->isOpen())
        return false;
    // 设置解析回调函数
    serial_ptr->setCallback(boost::bind(&DataParser::updateStatus, this, _1, _2));
    return true;
}

void DataParser::close() {
    serial_ptr->close();
}

// 解析回调函数
void DataParser::updateStatus(const char *data, unsigned int len) {
    for (int i = 0; i < len; ++i) {
        append(data[i]);
    }
}

/// 逐个字符解析
std::string buf_str;
char head[6] = {0};
int parity_num = 0;
bool start_parity = false;
void DataParser::append(const char c) {
    // printf("%c", c);
    /// 收到 $ 表示新消息开始，丢弃当前未完成的消息并重置
    if (c == '$') {
        for (int i = 0; i < 5; ++i)
            head[i] = head[i + 1];
        head[5] = c;
        buf_str = "";
        start_parity = false;
        parity_num = 0;
        return;
    }
    /// KSXT 解析
    if (head[0] == '$' && head[1] == 'K' && head[2] == 'S' && head[3] == 'X' && head[4] == 'T' && head[5] == ',') {
        buf_str.push_back(c);
        if (c == '*') {
            start_parity = true;
            parity_num++;
        } else if (start_parity) {
            parity_num++;
            if (parity_num == 3) {
                start_parity = false;
                parity_num = 0;
                for (int i = 0; i < 5; ++i)
                    head[i] = head[i + 1];
                head[5] = c;
                KSXT ksxt_msg;
                if (!validateKSXT(std::string("$KSXT,") + buf_str)) {
                    KSXT empty_msg;
                    rtk_callback(&empty_msg);
                }
                if (parseKSXT((std::string("$KSXT,") + buf_str).c_str(), ksxt_msg)) {
                    printf("KSXT: lat=%.6f lon=%.6f h=%.2f vel_e=%.2f vel_n=%.2f vel_u=%.2f heading=%.1f pos_q=%u hdg_q=%u\n",
                           ksxt_msg.data.lat, ksxt_msg.data.lon, ksxt_msg.data.height,
                           ksxt_msg.data.vel_east, ksxt_msg.data.vel_north, ksxt_msg.data.vel_up,
                           ksxt_msg.data.heading, ksxt_msg.data.pos_qual, ksxt_msg.data.heading_qual);
                    // 当 KSXT 航向为空(0)且无双天线定向时，使用 GPRMC course 作为 fallback
                    if (ksxt_msg.data.heading_qual != 4 && last_gprmc_valid_ && ksxt_msg.data.heading == 0) {
                        ksxt_msg.data.heading = last_gprmc_course_;
                    }
                    rtk_callback(&ksxt_msg);
                } else {
                    KSXT empty_msg;
                    rtk_callback(&empty_msg);
                }
            }
        }
    }
    /// GPRMC 解析：$GPRMC,time,status,lat,N/S,lon,E/W,speed,course,...
    else if (head[0] == 'G' && head[1] == 'P' && head[2] == 'R' && head[3] == 'M' && head[4] == 'C' && head[5] == ',') {
        buf_str.push_back(c);
        if (c == '*') {
            start_parity = true;
            parity_num++;
        } else if (start_parity) {
            parity_num++;
            if (parity_num == 3) {
                start_parity = false;
                parity_num = 0;
                for (int i = 0; i < 5; ++i)
                    head[i] = head[i + 1];
                head[5] = c;
                parseGPRMC((std::string("$GPRMC,") + buf_str).c_str());
            }
        }
    }
    else {
        for (int i = 0; i < 5; ++i) {
            head[i] = head[i + 1];
        }
        head[5] = c;
        buf_str = "";
        return;
    }
    return;
}


/// 解析KSXT语句
bool DataParser::parseKSXT(const char *ksxt_msg_in, KSXT &ksxt_msg_out) {
    const char* ksxt_begin = strstr(ksxt_msg_in, "$KSXT");
    // printf("%s\n", ksxt_msg_in);
    if (ksxt_begin == NULL) {
        return false;
    }

    // 使用 boost::split 分割字符串
    std::vector<std::string> tokens;
    boost::split(tokens, ksxt_msg_in, boost::is_any_of(","));

    if (tokens.size() < 20) { // 千寻格式至少需要 20 个字段（含 vel_up 在索引 19）
        printf("The message is too short, size: %zu\n", tokens.size());
        return false;
    }

    // 初始化结构体变量
    KSXT_UTC utc_time;
    KSXT_DATA ksxt;

    // 解析时间字段
    std::string time_str = tokens[1];
    // 时间字段长度应为 17（YYYYMMDDHHMMSS.SS）
    if (time_str.length() != 17) {
        printf("WARN, Invailed time: %s\n", time_str.c_str());
        return false;
    }
    /*
    utc_time.year = std::stoi(time_str.substr(0, 4));
    utc_time.month = std::stoi(time_str.substr(4, 2));
    utc_time.day = std::stoi(time_str.substr(6, 2));
    utc_time.hour = std::stoi(time_str.substr(8, 2));
    utc_time.minute = std::stoi(time_str.substr(10, 2));
    utc_time.second = std::stoi(time_str.substr(12, 2));
    utc_time.microsecond = std::stoi(time_str.substr(15, 2));
    */

    int year, month, day, hour, minute, second, microsecond;
    bool success = true;
    success &= safe_stoi(time_str.substr(0, 4), year);
    success &= safe_stoi(time_str.substr(4, 2), month);
    success &= safe_stoi(time_str.substr(6, 2), day);
    success &= safe_stoi(time_str.substr(8, 2), hour);
    success &= safe_stoi(time_str.substr(10, 2), minute);
    success &= safe_stoi(time_str.substr(12, 2), second);
    success &= safe_stoi(time_str.substr(15, 2), microsecond);

    if (!success) {
        std::cerr << "WARN, Failed to parse time fields: " << std::endl;
        return false;
    }

    utc_time.year = year;
    utc_time.month = month;
    utc_time.day = day;
    utc_time.hour = hour;
    utc_time.minute = minute;
    utc_time.second = second;
    utc_time.microsecond = microsecond * 10; //

    // 解析其他字段
    double temp_double = 0.0;
    ASSIGN_IF_VALID(LON_IDX,       ksxt.lon,       safe_stod)
    ASSIGN_IF_VALID(LAT_IDX,       ksxt.lat,       safe_stod)
    ASSIGN_IF_VALID(HEIGHT_IDX,    ksxt.height,    safe_stod)
    ASSIGN_IF_VALID(HEADING_IDX,   ksxt.heading,   safe_stod)
    ASSIGN_IF_VALID(PITCH_IDX,     ksxt.pitch,     safe_stod)
    ASSIGN_IF_VALID(VEL_IDX,       ksxt.vel,       safe_stod)
    ASSIGN_IF_VALID(ROLL_IDX,      ksxt.roll,      safe_stod)
    ASSIGN_IF_VALID(POS_QUAL_IDX,  ksxt.pos_qual,  safe_stoul)
    ASSIGN_IF_VALID(HEADING_QUAL_IDX, ksxt.heading_qual, safe_stoul)
    ASSIGN_IF_VALID(EAST_IDX,      ksxt.east,      safe_stod)
    ASSIGN_IF_VALID(NORTH_IDX,     ksxt.north,     safe_stod)
    ASSIGN_IF_VALID(UP_IDX,        ksxt.up,        safe_stod)

    /*
    if (safe_stod(tokens[VEL_EAST_IDX], temp_double)) {
        ksxt.vel_east = KMH_TO_MS(temp_double);
    }
    temp_double = 0.0;

    if (safe_stod(tokens[VEL_NORTH_IDX], temp_double)) {
        ksxt.vel_north = KMH_TO_MS(temp_double);
    }
    temp_double = 0.0;

    if (safe_stod(tokens[VEL_UP_IDX], temp_double)) {
        ksxt.vel_up = KMH_TO_MS(temp_double);
    }
    */

    // 处理速度字段（千寻格式：vel_east/north/up 单位 km/h，需转换为 m/s）
    double temp_vel;
    if (safe_stod(tokens[VEL_EAST_IDX], temp_vel)) ksxt.vel_east = KMH_TO_MS(temp_vel);
    if (safe_stod(tokens[VEL_NORTH_IDX], temp_vel)) ksxt.vel_north = KMH_TO_MS(temp_vel);
    if (safe_stod(tokens[VEL_UP_IDX], temp_vel)) ksxt.vel_up = KMH_TO_MS(temp_vel);

    // 检查经纬度是否有效
    // if (!isValidLatitudeLongitude(ksxt.lat, ksxt.lon)) {
    //     printf("Invalid latitude or longitude: lat=%lf, lon=%lf\n", ksxt.lat, ksxt.lon);
    //     return false;
    // }

    if (ksxt.lat < 18.0 || ksxt.lat > 53.5 || ksxt.lon < 73.5 || ksxt.lon > 135.0) {
        printf("WARN, Lon Lat value exceeds allowbale range, lat: %lf, lon: lf\n", ksxt.lat, ksxt.lon);
        return false;
    }

    // 输出调试信息
    /*printf("yyyymmddhhmmss.ss: %4d-%02d-%02d:%02d-%02d-%02d.%02d\n"
           "lon:%lf, lat:%lf, height:%lf\n"
           "heading:%lf, pitch:%lf, roll:%lf\n"
           "pos_qual:%d, heading_qual:%d\n"
           "east:%lf, north:%lf, up:%lf, vel:%lf\n"
           "vel_east:%lf, vel_north:%lf, vel_up:%lf\n",
           utc_time.year, utc_time.month, utc_time.day,
           utc_time.hour, utc_time.minute, utc_time.second, utc_time.microsecond,
           ksxt.lon, ksxt.lat, ksxt.height,
           ksxt.heading, ksxt.pitch, ksxt.roll,
           ksxt.pos_qual, ksxt.heading_qual,
           ksxt.east, ksxt.north, ksxt.up, ksxt.vel,
           ksxt.vel_east, ksxt.vel_north, ksxt.vel_up
          );*/

    // 填充输出结构体
    ksxt_msg_out.data = ksxt;
    ksxt_msg_out.utc = utc_time;

    return true;
}

/// 解析 GPRMC 语句，提取 course（航向）并缓存
/// $GPRMC,hhmmss.sss,A,llll.ll,a,yyyyy.yy,a,speed,course,ddmmyy,...
bool DataParser::parseGPRMC(const char* gprmc_msg_in) {
    const char* begin = strstr(gprmc_msg_in, "$GPRMC");
    if (begin == nullptr) return false;
    if (!validateKSXT(gprmc_msg_in)) return false;

    std::vector<std::string> tokens;
    boost::split(tokens, gprmc_msg_in, boost::is_any_of(","));
    if (tokens.size() < 9) return false;

    // status: A=有效, V=无效
    if (tokens[2].empty() || tokens[2][0] != 'A') {
        last_gprmc_valid_ = false;
        return false;
    }

    double course = 0.0;
    if (!safe_stod(tokens[8], course)) {
        last_gprmc_valid_ = false;
        return false;
    }
    last_gprmc_course_ = course;
    last_gprmc_valid_ = true;
    return true;
}

bool DataParser::parseKSXT2(const char *ksxt_msg_in, KSXT &ksxt_msg_out) {
    const char* ksxt_begin = strstr(ksxt_msg_in, "$KSXT");

    if (ksxt_begin == NULL) {
        return false;
    }

    // if (!ChecksumRmc(ksxt_begin, ksxt_msg_in + strlen(ksxt_msg_in))) {
    //     return false;
    // }
    KSXT_UTC utc_time;// = new KSXT_UTC;
    KSXT_DATA ksxt;// = new KSXT_DATA;
    // return false;
    // memset(&utc_time, 0, sizeof(KSXT_UTC));
    // memset(&ksxt, 0, sizeof(KSXT));
    // return false;
    int num = sscanf(ksxt_msg_in,
                    "$KSXT,%4hhu%2hhu%2hhu"
                    "%2hhu%2hhu%2hhu%*C%2hhu,"
                    "%lf,%lf,%lf,%lf,%lf,%*f,"
                    "%lf,%lf,%d,%d,%*[^,],%*[^,],"
                    "%lf,%lf,%lf,%lf,%lf,%lf",
                    &(utc_time.year),&(utc_time.month), &(utc_time.day),
                    &(utc_time.hour), &(utc_time.minute), &(utc_time.second), &(utc_time.microsecond),
                    &(ksxt.lon), &(ksxt.lat), &(ksxt.height), &(ksxt.heading), &(ksxt.pitch),
                    &(ksxt.vel), &(ksxt.roll), &(ksxt.pos_qual), &(ksxt.heading_qual),
                    &(ksxt.east), &(ksxt.north), &(ksxt.up), &(ksxt.vel_east), &(ksxt.vel_north), &(ksxt.vel_up)
                    );
    /*printf("yyyymmddhhmmss.ss: %d%d%d%d%d%d.%d\n"
        "lon:%lf, lat:%lf, height:%lf\n"
        "heading:%lf, pitch:%lf, roll:%lf\n"
        "pos_qual:%d, heading_qual:%d\n"
        "east:%lf, north:%lf, up:%lf, vel:%lf\n"
        "vel_east:%lf, vel_north:%lf, vel_up:%lf\n",
         utc_time.year, utc_time.month, utc_time.day,
         utc_time.hour, utc_time.minute, utc_time.second, utc_time.microsecond,
         ksxt.lon, ksxt.lat, ksxt.height,
         ksxt.heading, ksxt.pitch, ksxt.roll,
         ksxt.pos_qual, ksxt.heading_qual,
         ksxt.east, ksxt.north, ksxt.up, ksxt.vel,
         ksxt.vel_east, ksxt.vel_north, ksxt.vel_up
        );*/

    /*int num = sscanf(ksxt_msg_in,
                    "$KSXT,%4hhu%2hhu%2hhu"
                    "%2hhu%2hhu%2hhu%*C%2hhu,"
                    "%lf,%lf,%lf,%lf,%lf,%*f,"
                    "%lf,%lf,%d,%d,%*[^,],%*[^,],"
                    "%lf,%lf,%lf,%lf,%lf,%lf",
                    &(utc_time->year),&(utc_time->month), &(utc_time->day),
                    &(utc_time->hour), &(utc_time->minute), &(utc_time->second), &(utc_time->microsecond),
                    &(ksxt->lon), &(ksxt->lat), &(ksxt->height), &(ksxt->heading), &(ksxt->pitch),
                    &(ksxt->vel), &(ksxt->roll), &(ksxt->pos_qual), &(ksxt->heading_qual), &(ksxt->pos_qual),
                    &(ksxt->east), &(ksxt->north), &(ksxt->up), &(ksxt->vel_east), &(ksxt->vel_north), &(ksxt->vel_up)
                    );*/


    if (num != 22)
        return false;

    /*printf("yyyymmddhhmmss.ss: %d%d%d%d%d%d.%d\n"
           "lon:%lf, lat:%lf, height:%lf\n"
           "heading:%lf, pitch:%lf, roll:%lf\n"
           "east:%lf, north:%lf, up:%lf, vel:%lf\n"
           "vel_east:%lf, vel_north:%lf, vel_up:%lf\n",
            utc_time->year, utc_time->month, utc_time->day,
            utc_time->hour, utc_time->minute, utc_time->second, utc_time->microsecond,
            ksxt->lon, ksxt->lat, ksxt->height,
            ksxt->heading, ksxt->pitch, ksxt->roll,
            ksxt->east, ksxt->north, ksxt->up, ksxt->vel,
            ksxt->vel_east, ksxt->vel_north, ksxt->vel_up
           );*/

    ksxt.vel_east = KMH_TO_MS(ksxt.vel_east);
    ksxt.vel_north = KMH_TO_MS(ksxt.vel_north);
    ksxt.vel_up = KMH_TO_MS(ksxt.vel_up);

    ksxt_msg_out.data = ksxt;
    ksxt_msg_out.utc = utc_time;

    return true;
}
