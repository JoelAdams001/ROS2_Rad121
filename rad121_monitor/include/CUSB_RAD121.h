#ifndef CUSB_RAD121_H
#define CUSB_RAD121_H

#include <ftdi.h>
#include <chrono>
#include <deque>

class CUSB_RAD121
{
public:
    CUSB_RAD121();
    ~CUSB_RAD121();

    bool Open();
    bool Close();
    int ReadData(unsigned char* buffer, int size);
    double Calculate_CompensatedCPM(double CountsPerMinute);
    double Calculate_mRhr(double CountsPerMinute);

private:
    struct ftdi_context* ftHandle;
    std::chrono::steady_clock::time_point LastCountsTime;
};

#endif // CUSB_RAD121_H

