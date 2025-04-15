/**
 * @file robomas.cpp
 * @brief Implementation file for Robomas class and RobomasSender class.
 * @details This file contains the implementation of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-16
 * @version 0.0.1
 * 
 * @warning This code has not been tested yet.
 */

#include "robomas.hpp"

// RobomasSender class implementation
RobomasSender::RobomasSender(CAN& can) : _can(can), _robomas(nullptr) {
    _can.frequency(1000000); // 1Mbps
    _can.mode(CAN::Normal); // Normal mode
    _can.reset(); // Reset CAN controller
}
RobomasSender::~RobomasSender() = default;

void RobomasSender::SetRobomas(Robomas* robomas) {
    _robomas = robomas;
}
void RobomasSender::Start() {
    _can.attach(callback(this, &RobomasSender::Read), CAN::RxIrq);
}
bool RobomasSender::Send() {
    CANMessage msg;
    msg.id = (_robomas->GetID() <= 0x204) ? 0x200 : 0x1FF;
    msg.len = 8;
    msg.format = CANStandard;

    int16_t buff[4];
    _robomas->GetSendBuff(buff);

    msg.data[0] = (buff[0] >> 8) & 0xFF;
    msg.data[1] = buff[0] & 0xFF;
    msg.data[2] = (buff[1] >> 8) & 0xFF;
    msg.data[3] = buff[1] & 0xFF;
    msg.data[4] = (buff[2] >> 8) & 0xFF;
    msg.data[5] = buff[2] & 0xFF;
    msg.data[6] = (buff[3] >> 8) & 0xFF;
    msg.data[7] = buff[3] & 0xFF;

    return _can.write(msg);
}
void RobomasSender::Read() {
    CANMessage msg;
    msg.id = _robomas->GetID();
    msg.len = 8;
    msg.format = CANStandard;

    int16_t buff[4] = {0, 0, 0, 0};

    if (_can.read(msg)) {
        buff[0] = (msg.data[0] << 8) | msg.data[1];
        buff[1] = (msg.data[2] << 8) | msg.data[3];
        buff[2] = (msg.data[4] << 8) | msg.data[5];
        if (_robomas->GetMotorType() == MotorType::M3508) {
            buff[3] = (msg.data[6] << 8) | msg.data[7];
        }
        _robomas->SetReadData(buff);
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
Robomas::Robomas(RobomasSender& sender, MotorType type, uint8_t motor_num) : _sender(sender) {
    _sender.SetRobomas(this);
    _type = type;
    _id = motor_num + 0x200;
    _motor_num = motor_num;
    _torque_limit = 0;
    Init();
}

Robomas::~Robomas() = default;

// base functionality
void Robomas::GetSendBuff(int16_t* data) {
    memcpy(data, send_buff, sizeof(send_buff));
}
void Robomas::SetReadData(int16_t* data) {
    memcpy(read_data, data, sizeof(read_data));
}

// start/stop functionality
void Robomas::Init() {
    for (uint8_t i=0; i<(int)(sizeof(send_buff)/sizeof(send_buff[0])); i++) {
        send_buff[i] = 0;
    }
}

// set configure
void Robomas::SetMotorNum(uint8_t number) {
    _motor_num = number;
}
void Robomas::SetTorqueLimit(int16_t limit) {
    _torque_limit = limit;
}

// get configure
int Robomas::GetID() const {
    return _id;
}
int Robomas::GetMotorNum() const {
    return _motor_num;
}
MotorType Robomas::GetMotorType() const {
    return _type;
}
int16_t Robomas::GetTorqueLimit() const {
    return _torque_limit;
}

// main write
void Robomas::SetTorque(int16_t torque) {
    send_buff[this->GetMotorNum()-1] = torque;
}
void Robomas::SetSpeed(uint8_t speed) {
    // PIDを実装
}
void Robomas::SetPosition(int16_t position) {
    // PIDを実装
}

// main read
int16_t Robomas::GetMotorTorque() {
    return read_data[this->GetMotorNum()];
}
int16_t Robomas::GetMotorSpeed() {
    return read_data[this->GetMotorNum()];
}
int16_t Robomas::GetMotorPosition() {
    return read_data[this->GetMotorNum()];
}
// Robomas class implementation
// RobomasSender class implementation
