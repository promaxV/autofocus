#include "autofocus.hpp"

// MotorControl
MotorControl::MotorControl(const std::string& portName)
    : serial(io, portName) {
    while (serial.is_open() == false) {
        std::cerr << "Error: Could not open serial port " << portName << ".\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Параметры последовательного порта
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    std::this_thread::sleep_for(std::chrono::seconds(1)); // Чтобы убедиться, что порт открыт и настроен
    std::cout << "Serial port " << portName << " opened successfully.\n";
}

MotorControl::~MotorControl() {
    // Закрытие порта при завершении работы
    if (serial.is_open()) {
        serial.close();
        std::cout << "Serial port closed.\n";
    }
}

int MotorControl::getLastPosition(const unsigned char motorId) const {
    return lastPosition[motorId];
}

int MotorControl::getUnitSteps(const unsigned char motorId) const {
    return unitSteps[motorId];
}

unsigned long MotorControl::getMinSpeed(const unsigned char motorId) const {
    return minSpeed[motorId];
}

unsigned long MotorControl::getMaxSpeed(const unsigned char motorId) const {
    return maxSpeed[motorId];
}

unsigned long MotorControl::getAccel(const unsigned char motorId) const {
    return accel[motorId];
}

void MotorControl::sendCommand(const std::string& command) {
    if (serial.is_open()) {
        std::string fullCommand = command + "\r";
        boost::asio::write(serial, boost::asio::buffer(fullCommand));
        std::cout << "Sent command: " << fullCommand << std::endl;
    } else {
        std::cerr << "Error: Serial port is not open.\n";
    }
}

void MotorControl::moveSteps(const unsigned char motorId, const int steps) {
    std::string command = "s" + std::to_string(motorId) + ":" + std::to_string(-steps); // Минус нужен, чтобы положительные значения соответствовали отдалению фокуса и приближению зума
    sendCommand(command);
    // Задержка для ожидания завершения движения
    int delayMs = static_cast<int>(std::ceil(std::abs(steps) / static_cast<double>(minSpeed[motorId]) * 60000)) + 50; // Перевод скорости в минуты и расчет задержки
    lastPosition[motorId] += steps;
    std::cout << "Delay: " << delayMs << " ms\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
}

void MotorControl::moveToPosition(const unsigned char motorId, const int newPosition) {
    int steps = newPosition - lastPosition[motorId];
    moveSteps(motorId, steps);
}

// Поиграться со стандартными значениями
void MotorControl::setDefaultValues(const unsigned char motorId, 
                                    const unsigned int unitStepsNew, 
                                    const unsigned long minSpeedNew, 
                                    const unsigned long maxSpeedNew, 
                                    const unsigned long accelNew) {
    std::string command = "save" + std::to_string(motorId) + ":" +
                                   std::to_string(unitStepsNew) + "," + 
                                   std::to_string(minSpeedNew) + "," + 
                                   std::to_string(maxSpeedNew) + "," + 
                                   std::to_string(accelNew);
    sendCommand(command);
    unitSteps[motorId] = unitStepsNew;
    minSpeed[motorId] = minSpeedNew;
    maxSpeed[motorId] = maxSpeedNew;
    accel[motorId] = accelNew;
}

void MotorControl::initializeMotor(const unsigned char motorId, const int fullLength) {
    moveSteps(motorId, -fullLength);
    lastPosition[motorId] = 0;
}

// WEB CAMERA
WebCamera::WebCamera(const int cameraIndex) {
    camIndex = cameraIndex;
}

WebCamera::~WebCamera() {
    if (cap.isOpened()) {
        cap.release();
    }
}

void WebCamera::open() {
    std::cout << "Opening camera with index " << camIndex << std::endl;
    cap.open(camIndex, cv::CAP_DSHOW);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera with index " << camIndex << std::endl;
    } else {
        cameraOpened = true;
    }
    getFrame();
}

cv::Mat WebCamera::getFrame() {
    cv::Mat frame;
    if (cap.isOpened()) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Captured frame is empty." << std::endl;
        }
    } else {
        std::cerr << "Error: Camera is not opened." << std::endl;
    }
    return frame;
}

bool WebCamera::isOpened() const {
    return cap.isOpened();
}


// IMAGE PROCESSOR
double ImageProcessor::calculateLaplacianVariance(const cv::Mat& frame) {
    cv::Mat frameGray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);
    } else {
        frameGray = frame;
    }
    cv::Mat laplacian;
    cv::Laplacian(frameGray, laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    return (stddev.val[0] * stddev.val[0])*100000/frameGray.size().area();
}


// FOCUS CONTROLLER
FocusController::FocusController(MotorControl& motor, ICamera& camera, ImageProcessor& processor)
    : motor(motor), camera(camera), imageProcessor(processor), maxPosition(0), maxLaplacianValue(0.0) {
}

void FocusController::timeoutThread() {
    timeoutFlag = false;
    while (!cameraReady) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Ожидание готовности камеры
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(TIMEOUT_MS));
    if (!stopFlag) {
        timeoutFlag = true;
        std::cout << "Timeout reached.\n";
    }
}

void FocusController::captureThread() {
    if (!camera.isOpened()) {
        camera.open();
    }
    cameraReady = camera.isOpened();

    cv::namedWindow("Frame");

    while (!stopFlag && !timeoutFlag) {
        cv::Mat frame = camera.getFrame();
        {
            std::lock_guard<std::mutex> lock(mtx);
            frames.push_back(frame);
            if (frames.size() > 10) {                // Ограничение размера массива
                frames.erase(frames.begin());
            }
        }

        cv::imshow("Frame", frame);
        cv::waitKey(30);
    }

    cv::destroyWindow("Frame");
}

cv::Mat FocusController::getLastFrame() {
    std::lock_guard<std::mutex> lock(mtx);
    if (!frames.empty()) {
        return frames.back();
    }
    return cv::Mat();
}

void FocusController::saveFrame(const cv::Mat& frame, const std::string& filename) {
    if (!std::filesystem::exists("frames")) {
        std::filesystem::create_directory("frames");
    }
    std::string filepath = "frames/" + filename;
    cv::imwrite(filepath, frame);
}

void FocusController::focusAdjustment(const int steps, const unsigned int N, const double threshold, const int maxPositionShift) {
    unsigned stepsSinceMax = 0;
    maxLaplacianValue = 0.0;
    
    while (!timeoutFlag) {
        cv::Mat frame = getLastFrame();
        double laplacianVar = imageProcessor.calculateLaplacianVariance(frame);
        std::cout << "Laplacian variance: " << laplacianVar << " at focus position " << motor.getLastPosition(1) << std::endl;

        // std::string filename = "image_" + std::to_string(stepCounter) + ".png";
        // saveFrame(frame, filename);

        if (laplacianVar > maxLaplacianValue) {
            maxLaplacianValue = laplacianVar;
            maxPosition = motor.getLastPosition(1);
            stepsSinceMax = 0;
        } else {
            stepsSinceMax++;
        }

        if (stepsSinceMax >= N && maxLaplacianValue > threshold) {
            std::cout << "Optimal focus found at position: " << maxPosition << "\n";

            // std::string bestFilename = "image_best_" + std::to_string(stepCounter) + ".png";
            // saveFrame(frame, bestFilename);
            
            motor.moveSteps(1, -steps*(stepsSinceMax - maxPositionShift));
            return;
        }

        motor.moveSteps(1, steps);
        stepCounter+=steps;
    }
}

void FocusController::motorControlThread() {
    stopFlag = false;

    motor.setDefaultValues(1, 1, 3*360000UL, 3*1140000UL, 3*10000UL); // TODO поиграться со значениями для ускорения
    motor.initializeMotor(1);

    while (!cameraReady) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Ожидание готовности камеры
    }

    focusAdjustment(500, 15, 10, -1); // Первый проход с большим шагом

    std::cout << "Starting fine-tuning focus...\n";
    focusAdjustment(50, 30, 10); // Дополнительный проход с меньшим шагом
    stopFlag = true;
}

void FocusController::startFocusing() {
    autoFocusStarted = true;
    timeoutFlag = false;
    stopFlag = false;

    std::thread timeout(&FocusController::timeoutThread, this);
    std::thread capture(&FocusController::captureThread, this);
    std::thread motorControl(&FocusController::motorControlThread, this);
    
    capture.join();
    motorControl.join();
    timeout.join();

    autoFocusStarted = false;
}

bool FocusController::isTimeout() const {
    return timeoutFlag;
}

void FocusController::run() {
    if (!camera.isOpened()) {
        camera.open();
    }

    cv::namedWindow("Frame");

    while (true) {
        cv::Mat frame = camera.getFrame();
        int key = cv::waitKey(30);
        cv::imshow("Frame", frame);
        
        if (key == 27) { // ESC key
            break;
        } else if (key == 32 && !autoFocusStarted) { // SPACE key
            cv::destroyWindow("Frame");
            startFocusing();
            cv::namedWindow("Frame");
        }
    }
}

void startAutoFocus(const char* port, int cameraIndex) {
    // Инициализация компонентов
    MotorControl motor(port);
    WebCamera camera(cameraIndex);
    ImageProcessor imageProcessor;

    FocusController focusController(motor, camera, imageProcessor);

    focusController.run();
}

void testDllExport(const char *someString, int someInt)
{
    std::cout << "DLL export test: " << someString << " " << someInt << std::endl;
}

void moveMotorSteps(const char* port, unsigned char motorId, int steps) {
    MotorControl motor(port);
    motor.moveSteps(motorId, steps);
}

void setMotorDefaultValues(const char* port, unsigned char motorId, unsigned int unitSteps, unsigned long minSpeed, unsigned long maxSpeed, unsigned long accel) {
    MotorControl motor(port);
    motor.setDefaultValues(motorId, unitSteps, minSpeed, maxSpeed, accel);
}

int main() {
    std::string port;
    std::cout << "Enter the serial port name: ";
    std::cin >> port;

    int camIndex;
    std::cout << "Enter the camera index: ";
    std::cin >> camIndex;

    testDllExport(port.c_str(), camIndex);
    startAutoFocus(port.c_str(), camIndex);

    return 0;
}
