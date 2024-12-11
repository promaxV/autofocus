#include "dllexport.h"
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>
#include <atomic>
#include <condition_variable>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <boost/asio.hpp>
// #include <boost/bind.hpp>

#include <filesystem>

class MotorControl {
private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;

    int lastPosition[2] = {0, 0};
    int unitSteps[2] = {0, 0};
    unsigned long minSpeed[2] = {0, 0};
    unsigned long maxSpeed[2] = {0, 0};
    unsigned long accel[2] = {0, 0};

    void sendCommand(const std::string& command);

    char readBuffer[128];

public:
    MotorControl(const std::string& portName);
    ~MotorControl();

    int getLastPosition(const unsigned char motorId) const;
    int getUnitSteps(const unsigned char motorId) const;
    unsigned long getMinSpeed(const unsigned char motorId) const;
    unsigned long getMaxSpeed(const unsigned char motorId) const;
    unsigned long getAccel(const unsigned char motorId) const;

    void setDefaultValues(const unsigned char motorId, 
                          const unsigned int unitStepsNew = 1, 
                          const unsigned long minSpeedNew = 380000, 
                          const unsigned long maxSpeedNew = 1140000, 
                          const unsigned long accelNew = 10000);

    void moveSteps(const unsigned char motorId, const int steps);
    void moveToPosition(const unsigned char motorId, int newPosition);
    void initializeMotor(const unsigned char motorId, const int fullLength = 38000);
};

class ICamera {
public:
    virtual ~ICamera() = default;
    virtual cv::Mat getFrame() = 0;
    virtual void open() = 0;
    virtual bool isOpened() const = 0;
};

class WebCamera : public ICamera {
private:
    int camIndex;
    cv::VideoCapture cap;
    std::atomic<bool> cameraOpened{false};

public:
    WebCamera(const int cameraIndex);
    ~WebCamera() override;

    cv::Mat getFrame() override;
    void open() override;
    bool isOpened() const override;
};

class ImageProcessor {
public:
    double calculateLaplacianVariance(const cv::Mat& frame);
};

class FocusController {
private:
    MotorControl& motor;
    ICamera& camera;
    ImageProcessor& imageProcessor;
    std::vector<double> laplacianValues;
    std::mutex mtx;
    std::atomic<bool> cameraReady{false};
    std::atomic<bool> stopFlag{false};
    std::atomic<bool> timeoutFlag{false};
    std::atomic<bool> autoFocusStarted{false};
    int maxPosition;
    double maxLaplacianValue;
    std::string statusMessage;
    std::vector<cv::Mat> frames;

    const unsigned int TIMEOUT_MS = 20000;
    const unsigned int CAPTURE_THREAD_DELAY_MS = 34; // TODO оптимизировать задержку

    unsigned int stepCounter = 0;

    void captureThread();
    void motorControlThread();
    void timeoutThread();
    void focusAdjustment(const int steps, const unsigned int N, const double threshold, const int maxPositionShift = 0);
    cv::Mat getLastFrame();
    void saveFrame(const cv::Mat& frame, const std::string& filename);

public:
    FocusController(MotorControl& motor, ICamera& camera, ImageProcessor& processor);

    void startFocusing();
    bool isTimeout() const;
    void run();
};

/**
 * @brief Запускает автофокусировку.
 * 
 * @param portName Имя порта для подключения.
 * @param cameraIndex Индекс камеры.
 */
extern "C" {
    DLL_EXPORT void startAutoFocus(const char* portName, int cameraIndex);
}

/**
 * @brief Перемещает мотор на заданное количество шагов.
 * 
 * @param portName Имя порта для подключения.
 * @param motorId Идентификатор мотора.
 * @param steps Количество шагов.
 */
extern "C" {
    DLL_EXPORT void moveMotorSteps(const char* portName, unsigned char motorId, int steps);
}

/**
 * @brief Устанавливает стандартные значения параметров для мотора.
 * 
 * @param portName Имя порта для подключения.
 * @param motorId Идентификатор мотора.
 * @param unitSteps Количество шагов за единицу хода.
 * @param minSpeed Минимальная скорость.
 * @param maxSpeed Максимальная скорость.
 * @param accel Ускорение.
 */
extern "C" {
    DLL_EXPORT void setMotorDefaultValues(const char* port, unsigned char motorId, unsigned int unitSteps, unsigned long minSpeed, unsigned long maxSpeed, unsigned long accel);
}

extern "C" {
    DLL_EXPORT void testDllExport(const char* someString, int someInt);
}