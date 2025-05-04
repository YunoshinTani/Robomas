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
RobomasSender::RobomasSender(CAN& can, int can_frequency) : can_(can), robomas_{nullptr} {
    can_frequency_ = can_frequency;
}

RobomasSender::~RobomasSender() = default;

void RobomasSender::InitCan() {
    can_.frequency(can_frequency_); // 1Mbps
    can_.mode(CAN::Normal); // Normal mode
    // Note: frequency 入れると動かないかも
}
void RobomasSender::SetRobomas(Robomas* robomas, uint8_t robomas_num) {
    robomas_ = robomas;
    robomas_num_ = robomas_num;
}
void RobomasSender::ReadStart() {
    can_.attach(callback(this, &RobomasSender::Read), CAN::RxIrq);
}
bool RobomasSender::Send() {
    CANMessage send_msg;
    send_msg.id = 0x200;
    send_msg.len = 8;
    send_msg.format = CANStandard;
    int16_t buff[4] = {0, 0, 0, 0};
    for (uint8_t j=0; j<robomas_num_; j++) {
        buff[robomas_[j].GetMotorNum() - 1] = robomas_[j].GetSendBuff();
    }
    send_msg.data[0] = (buff[0] >> 8) & 0xFF;
    send_msg.data[1] = buff[0] & 0xFF;
    send_msg.data[2] = (buff[1] >> 8) & 0xFF;
    send_msg.data[3] = buff[1] & 0xFF;
    send_msg.data[4] = (buff[2] >> 8) & 0xFF;
    send_msg.data[5] = buff[2] & 0xFF;
    send_msg.data[6] = (buff[3] >> 8) & 0xFF;
    send_msg.data[7] = buff[3] & 0xFF;
    return can_.write(send_msg); // Send message;
}
void RobomasSender::Read() {
    int16_t buff[4] = {0, 0, 0, 0};
    CANMessage read_msg;
    if (can_.read(read_msg)) {
        buff[0] = (int16_t)((read_msg.data[0] << 8) | read_msg.data[1]);
        buff[1] = (int16_t)((read_msg.data[2] << 8) | read_msg.data[3]);
        buff[2] = (int16_t)((read_msg.data[4] << 8) | read_msg.data[5]);
        buff[3] = (int16_t)read_msg.data[6];
        robomas_[read_msg.id - 0x200 - 1].SetReadData(buff); // Set read data to the motor
    }
}
void RobomasSender::Debug() {
    CANMessage read_msg;
    if (can_.read(read_msg)) {
        printf("TD:%3d,   ", can_.tderror()); // write error count
        printf("RD:%3d,   ", can_.rderror()); // read error count
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
    return can_.tderror();
}
uint8_t RobomasSender::GetReadError() {
    return can_.rderror();
}
void RobomasSender::CanReset() {
    can_.reset();
}

// Robomas class implementation
Robomas::Robomas(MotorType type, uint8_t motor_num, float Kp, float Ki, float Kd, std::chrono::milliseconds dt) {
    type_ = type;
    feedback_id_ = motor_num + 0x200;
    motor_num_ = motor_num;
    max_current_ = (type == MotorType::M2006) ? M2006::MAX_CURRENT : M3508::MAX_CURRENT;
    max_rpm_ = (type == MotorType::M2006) ? M2006::MAX_RPM : M3508::MAX_RPM;
    ResetData();
    SetPidGain(Kp, Ki, Kd, dt); // Set PID gain
    ResetPid(); // ReSet PID values
    now_time_ = HighResClock::now().time_since_epoch().count() / 1e6; // Get current time
    prev_time_ = now_time_; // Get previous time
    sec_dt_ = static_cast<float>(dt.count()) / 1000.0f; // Convert milliseconds to seconds
}

Robomas::~Robomas() = default;

int16_t Robomas::GetSendBuff() {
    return send_buff;
}
void Robomas::SetReadData(int16_t* data) {
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = data[i];
    }
}
void Robomas::ResetData() {
    send_buff = 0;
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = 0;
    }
    SetCurrent(0); // Set current to 0
}
void Robomas::SetMotorNum(uint8_t number) {
    motor_num_ = number;
    feedback_id_ = number + 0x200;
}
void Robomas::SetMotorType(MotorType type)  {
    type_ = type;
}
void Robomas::SetCurrentLimit(uint16_t limit) {
    max_current_ = limit;
}
void Robomas::SetTorqueLimit(uint16_t limit) {
    SetCurrentLimit(limit / (type_ == MotorType::M2006 ? M2006::TORQUE_CONSTANT : M3508::TORQUE_CONSTANT)); // Set torque limit
}
void Robomas::SetRpmLimit(uint16_t limit) {
    max_rpm_ = limit;
}
void Robomas::SetPidGain(float Kp, float Kd, float Ki, std::chrono::milliseconds dt) {
    Kp_ = Kp;
    Kd_ = Kd;
    Ki_ = Ki;
    sec_dt_ = static_cast<float>(dt.count()) / 1000.0f;  // Convert milliseconds to seconds
}
void Robomas::ResetPid() {
    current_ = 0;
    target_ = 0;
    output_max_ = 0;
    error_ = 0.0f;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    prev_error_ = 0.0f;
    output_ = 0.0f;
}
void Robomas::Pid(int16_t target, int16_t current, uint16_t max) {
    target_ = target;
    current_ = current;
    output_max_ = max;
    UpdatePid();
}
void Robomas::UpdatePid() {
    now_time_ = HighResClock::now().time_since_epoch().count() / 1e6; // Get current time
    if (now_time_ - prev_time_ >= sec_dt_) {
        error_ = target_ - current_;
        integral_ += error_ * sec_dt_; // Integral term
        derivative_ = (error_ - prev_error_) / sec_dt_;
        output_ = (int16_t)(Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative_);
        if (output_ > output_max_) output_ = output_max_;
        else if (output_ < -output_max_) output_ = -output_max_;
        prev_error_ = error_;
        SetTorque(output_);
        prev_time_ = now_time_; // Update previous time
    }
    // printf("current:%5d,   ", current_);
    // printf("error:%5.2f,   ", error_); // Debug output
    // printf("integral:%5.2f,   ", integral_); // Debug output
    // printf("derivative:%5.2f,   ", derivative_); // Debug output
    // printf("output:%5d,   ", output_);
}
uint16_t Robomas::GetReadId() const {
    return feedback_id_;
}
uint8_t Robomas::GetMotorNum() const {
    return motor_num_;
}
MotorType Robomas::GetMotorType() const {
    return type_;
}
uint16_t Robomas::GetCurrentLimit() const {
    return max_current_;
}
uint16_t Robomas::GetTorqueLimit() const {
    return max_current_ * (type_ == MotorType::M2006 ? M2006::TORQUE_CONSTANT : M3508::TORQUE_CONSTANT); // Get torque limit
}
uint16_t Robomas::GetRpmLimit() const {
    return max_rpm_;
}
void Robomas::SetCurrent(int16_t current) {
    if (current > max_current_) {
        current = max_current_;
    } else if (current < -max_current_) {
        current = -max_current_;
    }
    send_buff = current;
}
void Robomas::SetTorque(int16_t torque) {
    if (type_ == MotorType::M2006) {
        SetCurrent(torque / M2006::TORQUE_CONSTANT);
    } else if (type_ == MotorType::M3508) {
        SetCurrent(torque / M3508::TORQUE_CONSTANT);
    }
}
void Robomas::SetRpm(int16_t target_rpm) {
    Pid(target_rpm, GetRpm(), max_rpm_); // Set tarGet rpm and current rpm
}
void Robomas::SetPosition(int16_t target_position) {
    Pid(target_position, GetPosition(), max_rpm_); // Set tarGet position and current position
}
void Robomas::SetBrake() {
    SetRpm(0); // Set RPM to 0
}
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
    if (type_ == MotorType::M3508) {
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
