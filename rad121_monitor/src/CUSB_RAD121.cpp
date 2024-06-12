#include "CUSB_RAD121.h"
#include <iostream>
#include <cmath>
#include <ctime>

#define USB_RAD121_SENSITIVITY 450
#define USB_RAD121_DEAD_TIME 0.00015

CUSB_RAD121::CUSB_RAD121()
{
    ftHandle = ftdi_new();
}

CUSB_RAD121::~CUSB_RAD121()
{
    Close();
    ftdi_free(ftHandle);
}

bool CUSB_RAD121::Open()
{
    if (ftdi_usb_open(ftHandle, 0x0403, 0x6001) < 0)
    {
        std::cerr << "Failed to open FTDI device" << std::endl;
        return false;
    }

    ftdi_set_baudrate(ftHandle, 921600);
    ftdi_set_line_property(ftHandle, BITS_8, STOP_BIT_1, NONE);
    ftdi_usb_purge_buffers(ftHandle);
    LastCountsTime = std::chrono::steady_clock::now();
    std::cerr << "FTDI device opened successfully" << std::endl;

    return true;
}

bool CUSB_RAD121::Close()
{
    if (ftdi_usb_close(ftHandle) < 0)
    {
        std::cerr << "Failed to close FTDI device" << std::endl;
        return false;
    }
    return true;
}

int CUSB_RAD121::ReadData(unsigned char* buffer, int size)
{
    return ftdi_read_data(ftHandle, buffer, size);
}

double CUSB_RAD121::Calculate_CompensatedCPM(double CountsPerMinute)
{
    return CountsPerMinute / (1.0 - (CountsPerMinute / 60.0) * USB_RAD121_DEAD_TIME);
}

double CUSB_RAD121::Calculate_mRhr(double CountsPerMinute)
{
    return CountsPerMinute / USB_RAD121_SENSITIVITY;
}

