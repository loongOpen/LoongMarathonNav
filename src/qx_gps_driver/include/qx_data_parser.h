//
//

#ifndef DATAPARSER_H
#define DATAPARSER_H

#include "qx_async_serial.h"
#include "type.h"

class DataParser {
public:
    DataParser(const std::string &devname, unsigned int& baud_rate);

    ~DataParser();

    void setCallback(const boost::function<void(const KSXT* msg)> &call_back);

    bool run_parse();

    void close();

private:
    void updateStatus(const char *data, unsigned int len);
    void append(const char c);
    bool parseKSXT(const char* ksxt_msg_in, KSXT &ksxt_msg_out);
    bool parseKSXT2(const char* ksxt_msg_in, KSXT &ksxt_msg_out);
    bool parseGPRMC(const char* gprmc_msg_in);

    boost::function<void(const KSXT* msg)> rtk_callback;

    // GPRMC 解析的 course（航向）缓存，当 KSXT 无有效航向时作为 fallback
    double last_gprmc_course_;
    bool last_gprmc_valid_;

    std::shared_ptr<CallbackAsyncSerial> serial_ptr;

    std::string serial_name;
    unsigned int serial_baud;

};



#endif //DATAPARSER_H
