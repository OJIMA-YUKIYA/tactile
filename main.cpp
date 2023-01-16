#include <fstream>
#include <ctime>
#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#include <cstring>

#include <sys/fcntl.h>
#include <iomanip>

#include <arpa/inet.h>
#include <unistd.h>

#include <termios.h>

#include "serialPathConfig.h" // ESP32_SERIAL_PATH, MOMO_SERIAL_PATH を定義


// const uint8_t PC_ESP32_MEASURE_REQUEST[4] = {0x00, 0x00, 0x00, 0xFE};
const int SENSOR_X_NUM = 10;
const int SENSOR_Y_NUM = 10;
const int THERMAL_NUM = 2;
const int FINGER_NUM = 3;
const int SENSOR_NUM = SENSOR_X_NUM * SENSOR_Y_NUM * FINGER_NUM;

int PressureDistribution[FINGER_NUM][SENSOR_X_NUM][SENSOR_Y_NUM] = {0};
const int DATA_LENGTH = (int)(FINGER_NUM * (SENSOR_X_NUM * SENSOR_Y_NUM + THERMAL_NUM) * 3.0 / 2.0) + 2;

uint8_t m_Rcv[DATA_LENGTH+1] = {0};

void esp32_serial_thread() {
    int esp32_serial_port = open(ESP32_SERIAL_PATH, O_RDWR);
    int momo_serial_port = open(MOMO_SERIAL_PATH, O_WRONLY);
    printf("esp32 port idx:%d, DATA_LENGTH: %d\n", esp32_serial_port, DATA_LENGTH);
    printf("momo port idx:%d\n", momo_serial_port);
    struct termios tty;
    // Read in existing settings, and handle any error
    if (tcgetattr(esp32_serial_port, &tty) != 0) {
        // printf("Error %i from tcgetattr: %s\n", errno, perror(errno));
        return;
    }
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 1; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 921600
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);

    // Save tty settings, also checking for error
    if (tcsetattr(esp32_serial_port, TCSANOW, &tty) != 0) {
        // printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return;
    }

    uint8_t PC_ESP32_MEASURE_REQUEST[2] = {0xFE, '\r'};

    write(esp32_serial_port, &PC_ESP32_MEASURE_REQUEST, 2);
    int count = 0;
    int begin_idx = 0;
    printf("data length: %d\n", DATA_LENGTH);
    while (true) {
        int req_size = DATA_LENGTH*sizeof(uint8_t);
        uint8_t data[DATA_LENGTH] = {0};

        int read_size = read(esp32_serial_port, data, req_size);
        if (read_size < 1) {
            begin_idx = 0;
            m_Rcv[DATA_LENGTH] = 10;
            write(momo_serial_port, m_Rcv, DATA_LENGTH+1);
            memset(m_Rcv, 0, DATA_LENGTH);
            printf("request %d\n", count++);
            write(esp32_serial_port, &PC_ESP32_MEASURE_REQUEST, 2);

            continue;
        }
        printf("read %d byte\n", read_size);
        for (int idx = 0; idx < read_size; idx++) {
            m_Rcv[begin_idx + idx] = data[idx];
        }
        begin_idx += read_size;

    }
    return;
}

void hoge() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return;
}


int main(int argc, char **argv) {
    std::thread th_esp32_serial_thread(esp32_serial_thread);

    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    // std::thread th_hoge(hoge);

    return 0;
}
