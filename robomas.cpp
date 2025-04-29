/**
 * @file robomas.cpp
 * @brief Implementation file for Robomas class and RobomasSender class.
 * @details This file contains the implementation of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-29
 * @version 3.3.0
 */

#include "robomas.hpp"

// RobomasSender class implementation
RobomasSender::RobomasSender(CAN& can, int can_frequency) : _can(can), _robomas{nullptr} {
    _can_frequency = can_frequency;
}

RobomasSender::~RobomasSender() = default;

void RobomasSender::initCan() {
    _can.frequency(_can_frequency); // 1Mbps
    _can.mode(CAN::Normal); // Normal mode
    // Note: frequency 入れると動かないかも
}
void RobomasSender::setRobomas(Robomas* robomas, uint8_t robomas_num) {
    _robomas = robomas;
    _robomas_num = robomas_num;
}
void RobomasSender::readStart() {
    _can.attach(callback(this, &RobomasSender::read), CAN::RxIrq);
}
bool RobomasSender::send() {
    CANMessage send_msg;
    send_msg.id = 0x200;
    send_msg.len = 8;
    send_msg.format = CANStandard;
    int16_t buff[4] = {0, 0, 0, 0};
    for (uint8_t j=0; j<_robomas_num; j++) {
        buff[_robomas[j].getMotorNum() - 1] = _robomas[j].getSendBuff();
    }
    send_msg.data[0] = (buff[0] >> 8) & 0xFF;
    send_msg.data[1] = buff[0] & 0xFF;
    send_msg.data[2] = (buff[1] >> 8) & 0xFF;
    send_msg.data[3] = buff[1] & 0xFF;
    send_msg.data[4] = (buff[2] >> 8) & 0xFF;
    send_msg.data[5] = buff[2] & 0xFF;
    send_msg.data[6] = (buff[3] >> 8) & 0xFF;
    send_msg.data[7] = buff[3] & 0xFF;
    return _can.write(send_msg); // Send message;
}
void RobomasSender::read() {
    int16_t buff[4] = {0, 0, 0, 0};
    CANMessage read_msg;
    if (_can.read(read_msg)) {
        buff[0] = (int16_t)((read_msg.data[0] << 8) | read_msg.data[1]);
        buff[1] = (int16_t)((read_msg.data[2] << 8) | read_msg.data[3]);
        buff[2] = (int16_t)((read_msg.data[4] << 8) | read_msg.data[5]);
        buff[3] = (int16_t)read_msg.data[6];
        _robomas[read_msg.id - 0x200 - 1].setReadData(buff); // Set read data to the motor
    }
}
void RobomasSender::debug() {
    CANMessage read_msg;
    if (_can.read(read_msg)) {
        printf("TD:%3d,   ", _can.tderror()); // write error count
        printf("RD:%3d,   ", _can.rderror()); // read error count
        printf("ID: 0x%3x,   ", read_msg.id);
        printf("Position:%4d,   ", (int16_t)((read_msg.data[0] << 8) | read_msg.data[1]));
        printf("Velocity:%5d,   ", (int16_t)((read_msg.data[2] << 8) | read_msg.data[3]));
        printf("Torque:%5d,   ",   (int16_t)((read_msg.data[4] << 8) | read_msg.data[5]));
        printf("Temperature:%2d,       ", (int16_t)read_msg.data[6]);
        printf("0:%2d,   ", (uint8_t)read_msg.data[0]);
        printf("1:%3d,   ", (uint8_t)read_msg.data[1]);
        printf("2:%3d,   ", (uint8_t)read_msg.data[2]);
        printf("3:%3d,   ", (uint8_t)read_msg.data[3]);
        printf("4:%3d,   ", (uint8_t)read_msg.data[4]);
        printf("5:%3d,   ", (uint8_t)read_msg.data[5]);
        printf("6:%2d,   ", (uint8_t)read_msg.data[6]);
        printf("7:%2d,   ", (uint8_t)read_msg.data[7]);
        // Note: data[7] is Null
        printf("\n");
    }
    else {
        printf("No message\r");
    }
}
uint8_t RobomasSender::getWriteError() {
    return _can.tderror();
}
uint8_t RobomasSender::getReadError() {
    return _can.rderror();
}
void RobomasSender::canReset() {
    _can.reset();
}

// Robomas class implementation
Robomas::Robomas(MotorType type, uint8_t motor_num, float Kp, float Ki, float Kd, std::chrono::milliseconds dt) {
    _type = type;
    _feedback_id = motor_num + 0x200;
    _motor_num = motor_num;
    _max_current = (type == MotorType::M2006) ? M2006::MAX_CURRENT : M3508::MAX_CURRENT;
    _max_rpm = (type == MotorType::M2006) ? M2006::MAX_RPM : M3508::MAX_RPM;
    resetData();
    setPidGain(Kp, Ki, Kd, dt); // Set PID gain
    resetPid(); // Reset PID values
    _now_time = HighResClock::now().time_since_epoch().count() / 1e6; // Get current time
    _prev_time = _now_time; // Get previous time
    _sec_dt = static_cast<float>(dt.count()) / 1000.0f; // Convert milliseconds to seconds
}

Robomas::~Robomas() = default;

int16_t Robomas::getSendBuff() {
    return send_buff;
}
void Robomas::setReadData(int16_t* data) {
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = data[i];
    }
}
void Robomas::resetData() {
    send_buff = 0;
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = 0;
    }
    setCurrent(0); // Set current to 0
}
void Robomas::setMotorNum(uint8_t number) {
    _motor_num = number;
    _feedback_id = number + 0x200;
}
void Robomas::setMotorType(MotorType type)  {
    _type = type;
}
void Robomas::setCurrentLimit(uint16_t limit) {
    _max_current = limit;
}
void Robomas::setTorqueLimit(uint16_t limit) {
    setCurrentLimit(limit / (_type == MotorType::M2006 ? M2006::TORQUE_CONSTANT : M3508::TORQUE_CONSTANT)); // Set torque limit
}
void Robomas::setRpmLimit(uint16_t limit) {
    _max_rpm = limit;
}
void Robomas::setPidGain(float Kp, float Kd, float Ki, std::chrono::milliseconds dt) {
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _sec_dt = static_cast<float>(dt.count()) / 1000.0f;  // Convert milliseconds to seconds
}
void Robomas::resetPid() {
    _current = 0;
    _target = 0;
    _output_max = 0;
    _error = 0.0f;
    _integral = 0.0f;
    _derivative = 0.0f;
    _prev_error = 0.0f;
    _output = 0.0f;
}
void Robomas::pid(int16_t target, int16_t current, uint16_t max) {
    _target = target;
    _current = current;
    _output_max = max;
    updatePid();
}
void Robomas::updatePid() {
    _now_time = HighResClock::now().time_since_epoch().count() / 1e6; // Get current time
    if (_now_time - _prev_time >= _sec_dt) {
        _error = _target - _current;
        _integral += _error * _sec_dt; // Integral term
        _derivative = (_error - _prev_error) / _sec_dt;
        _output = (int16_t)(_Kp * _error + _Ki * _integral + _Kd * _derivative);
        if (_output > _output_max) _output = _output_max;
        else if (_output < -_output_max) _output = -_output_max;
        _prev_error = _error;
        setTorque(_output);
        _prev_time = _now_time; // Update previous time
    }
    // printf("current:%5d,   ", _current);
    // printf("error:%5.2f,   ", _error); // Debug output
    // printf("integral:%5.2f,   ", _integral); // Debug output
    // printf("derivative:%5.2f,   ", _derivative); // Debug output
    // printf("output:%5d,   ", _output);
}
uint16_t Robomas::getReadId() const {
    return _feedback_id;
}
uint8_t Robomas::getMotorNum() const {
    return _motor_num;
}
MotorType Robomas::getMotorType() const {
    return _type;
}
uint16_t Robomas::getCurrentLimit() const {
    return _max_current;
}
uint16_t Robomas::getTorqueLimit() const {
    return _max_current * (_type == MotorType::M2006 ? M2006::TORQUE_CONSTANT : M3508::TORQUE_CONSTANT); // Get torque limit
}
uint16_t Robomas::getRpmLimit() const {
    return _max_rpm;
}
void Robomas::setCurrent(int16_t current) {
    if (current > _max_current) {
        current = _max_current;
    } else if (current < -_max_current) {
        current = -_max_current;
    }
    send_buff = current;
}
void Robomas::setTorque(int16_t torque) {
    if (_type == MotorType::M2006) {
        setCurrent(torque / M2006::TORQUE_CONSTANT);
    } else if (_type == MotorType::M3508) {
        setCurrent(torque / M3508::TORQUE_CONSTANT);
    }
}
void Robomas::setRpm(int16_t target_rpm) {
    pid(target_rpm, getRpm(), _max_rpm); // Set target rpm and current rpm
}
void Robomas::setPosition(int16_t target_position) {
    pid(target_position, getPosition(), _max_rpm); // Set target position and current position
}
void Robomas::setBrake() {
    setRpm(0); // Set RPM to 0
}
uint16_t Robomas::getPosition() {
    return read_data[0];
}
int Robomas::getTotalPosition() {
    // 角度の合計値
    return 0;
}
int16_t Robomas::getRpm() {
    return read_data[1];
}
int16_t Robomas::getTorque() {
    return read_data[2];
}
uint8_t Robomas::getTemperature() {
    if (_type == MotorType::M3508) {
        return read_data[3];
    }
    else {
        printf("Robomas::GetTemperature: 'This function is not supported for this motor type.'\n");
        return 0;
    }
}
void Robomas::debug() {
    printf("Pos:%4d,   ", getPosition());    // Get motor position
    printf("RPM:%5d,   ", getRpm());         // Get motor RPM
    printf("Tor:%5d,   ", getTorque());      // Get motor torque(current)
    printf("Tem:%2d,   ", getTemperature()); // Get motor temperature
}

// Robomas class implementation
// RobomasSender class implementation
