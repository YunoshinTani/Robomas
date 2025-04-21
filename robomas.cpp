/**
 * @file robomas.cpp
 * @brief Implementation file for Robomas class and RobomasSender class.
 * @details This file contains the implementation of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-22
 * @version 3.2.0
 */

#include "robomas.hpp"

// RobomasSender class implementation
RobomasSender::RobomasSender(CAN& can, int can_frequency) : _can(can), _robomas{nullptr} {
    _can_frequency = can_frequency;
}

RobomasSender::~RobomasSender() = default;

void RobomasSender::InitCan() {
    _can.frequency(_can_frequency); // 1Mbps
    _can.mode(CAN::Normal); // Normal mode
}
// Note: frequency 入れると動かないかも

void RobomasSender::SetRobomas(Robomas* robomas, uint8_t robomas_num) {
    _robomas = robomas;
    _robomas_num = robomas_num;
}
void RobomasSender::ReadStart() {
    _can.attach(callback(this, &RobomasSender::Read), CAN::RxIrq);
}
bool RobomasSender::Send() {
    CANMessage send_msg;
    send_msg.id = 0x200;
    send_msg.len = 8;
    send_msg.format = CANStandard;
    int16_t buff[4] = {0, 0, 0, 0};
    for (uint8_t j=0; j<_robomas_num; j++) {
        buff[_robomas[j].GetMotorNum() - 1] = _robomas[j].GetSendBuff();
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
void RobomasSender::Read() {
    int16_t buff[4] = {0, 0, 0, 0};
    CANMessage read_msg;
    if (_can.read(read_msg)) {
        buff[0] = (int16_t)((read_msg.data[0] << 8) | read_msg.data[1]);
        buff[1] = (int16_t)((read_msg.data[2] << 8) | read_msg.data[3]);
        buff[2] = (int16_t)((read_msg.data[4] << 8) | read_msg.data[5]);
        buff[3] = (int16_t)read_msg.data[6];
        _robomas[read_msg.id - 0x200 - 1].SetReadData(buff); // Set read data to the motor
    }
}
void RobomasSender::Debug() {
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
uint8_t RobomasSender::GetWriteError() {
    return _can.tderror();
}
uint8_t RobomasSender::GetReadError() {
    return _can.rderror();
}
void RobomasSender::CanReset() {
    _can.reset();
}

// Robomas class implementation
Robomas::Robomas(MotorType type, uint8_t motor_num, float Kp, float Ki, float Kd, std::chrono::milliseconds dt) {
    _type = type;
    _feedback_id = motor_num + 0x200;
    _motor_num = motor_num;
    _max_current = (type == MotorType::M2006) ? M2006::MAX_CURRENT : M3508::MAX_CURRENT;
    _max_rpm = (type == MotorType::M2006) ? M2006::MAX_RPM : M3508::MAX_RPM;
    InitData();
    SetPidGain(Kp, Ki, Kd, dt); // Set PID gain
    ResetPid(); // Reset PID values
    _now_time = HighResClock::now().time_since_epoch().count() / 1e6; // Get current time
    _prev_time = _now_time; // Get previous time
    _sec_dt = static_cast<float>(dt.count()) / 1000.0f; // Convert milliseconds to seconds
}

Robomas::~Robomas() = default;

// base functionality
int16_t Robomas::GetSendBuff() {
    return send_buff;
}
void Robomas::SetReadData(int16_t* data) {
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = data[i];
    }
}

// init functionality
void Robomas::InitData() {
    send_buff = 0;
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = 0;
    }
    SetCurrent(0); // Set current to 0
}

// set configure
void Robomas::SetMotorNum(uint8_t number) {
    _motor_num = number;
    _feedback_id = number + 0x200;
}
void Robomas::SetMotorType(MotorType type)  {
    _type = type;
}
void Robomas::SetCurrentLimit(uint16_t limit) {
    _max_current = limit;
}
void Robomas::SetTorqueLimit(uint16_t limit) {
    SetCurrentLimit(limit / (_type == MotorType::M2006 ? M2006::TORQUE_CONSTANT : M3508::TORQUE_CONSTANT)); // Set torque limit
}
void Robomas::SetRpmLimit(uint16_t limit) {
    _max_rpm = limit;
}

// PID functionality
void Robomas::SetPidGain(float Kp, float Kd, float Ki, std::chrono::milliseconds dt) {
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _sec_dt = static_cast<float>(dt.count()) / 1000.0f;  // Convert milliseconds to seconds
}
void Robomas::ResetPid() {
    _current = 0;
    _target = 0;
    _output_max = 0;
    _error = 0.0f;
    _integral = 0.0f;
    _derivative = 0.0f;
    _prev_error = 0.0f;
    _output = 0.0f;
}
void Robomas::Pid(int16_t target, int16_t current, uint16_t max) {
    _target = target;
    _current = current;
    _output_max = max;
    UpdatePid();
}
void Robomas::UpdatePid() {
    _now_time = HighResClock::now().time_since_epoch().count() / 1e6; // Get current time
    if (_now_time - _prev_time >= _sec_dt) {
        _error = _target - _current;
        _integral += _error * _sec_dt; // Integral term
        _derivative = (_error - _prev_error) / _sec_dt;
        _output = (int16_t)(_Kp * _error + _Ki * _integral + _Kd * _derivative);
        if (_output > _output_max) _output = _output_max;
        else if (_output < -_output_max) _output = -_output_max;
        _prev_error = _error;
        SetTorque(_output);
        _prev_time = _now_time; // Update previous time
    }
    // printf("current:%5d,   ", _current);
    // printf("error:%5.2f,   ", _error); // Debug output
    // printf("integral:%5.2f,   ", _integral); // Debug output
    // printf("derivative:%5.2f,   ", _derivative); // Debug output
    // printf("output:%5d,   ", _output);
}

// get configure
uint16_t Robomas::GetReadId() const {
    return _feedback_id;
}
uint8_t Robomas::GetMotorNum() const {
    return _motor_num;
}
MotorType Robomas::GetMotorType() const {
    return _type;
}
uint16_t Robomas::GetCurrentLimit() const {
    return _max_current;
}
uint16_t Robomas::GetTorqueLimit() const {
    return _max_current * (_type == MotorType::M2006 ? M2006::TORQUE_CONSTANT : M3508::TORQUE_CONSTANT); // Get torque limit
}
uint16_t Robomas::GetRpmLimit() const {
    return _max_rpm;
}

// main write
void Robomas::SetCurrent(int16_t current) {
    if (current > _max_current) {
        current = _max_current;
    } else if (current < -_max_current) {
        current = -_max_current;
    }
    send_buff = current;
}
void Robomas::SetTorque(int16_t torque) {
    if (_type == MotorType::M2006) {
        SetCurrent(torque / M2006::TORQUE_CONSTANT);
    } else if (_type == MotorType::M3508) {
        SetCurrent(torque / M3508::TORQUE_CONSTANT);
    }
}
void Robomas::SetRpm(int16_t target_rpm) {
    Pid(target_rpm, GetRpm(), _max_rpm); // Set target rpm and current rpm
}
void Robomas::SetPosition(int16_t target_position) {
    Pid(target_position, GetPosition(), _max_rpm); // Set target position and current position
}
void Robomas::SetBrake() {
    SetRpm(0); // Set RPM to 0
}

// main read
uint16_t Robomas::GetPosition() {
    return read_data[0];
}
int Robomas::GetTotalPosition() {
    // 角度の合計値
    return 0;
}
int16_t Robomas::GetRpm() {
    return read_data[1];
}
int16_t Robomas::GetTorque() {
    return read_data[2];
}
uint8_t Robomas::GetTemperature() {
    if (_type == MotorType::M3508) {
        return read_data[3];
    }
    else {
        printf("Robomas::GetTemperature: 'This function is not supported for this motor type.'\n");
        return 0;
    }
}
void Robomas::Debug() {
    printf("Pos:%4d,   ", GetPosition());    // Get motor position
    printf("RPM:%5d,   ", GetRpm());         // Get motor RPM
    printf("Tor:%5d,   ", GetTorque());      // Get motor torque(current)
    printf("Tem:%2d,   ", GetTemperature()); // Get motor temperature
}

// Robomas class implementation
// RobomasSender class implementation
