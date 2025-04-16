/**
 * @file robomas.cpp
 * @brief Implementation file for Robomas class and RobomasSender class.
 * @details This file contains the implementation of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-17
 * @version 2.0.0
 * 
 * @warning This code has not been tested yet.
 */

#include "robomas.hpp"

// RobomasSender class implementation
RobomasSender::RobomasSender(CAN& can) : _can(can), _robomas{nullptr} {
    InitCan(); // Initialize CAN communication
}

RobomasSender::~RobomasSender() = default;

void RobomasSender::InitCan() {
    _can.frequency(CAN_FREQUENCY); // 1Mbps
    _can.mode(CAN::Normal); // Normal mode
    _can.reset(); // Reset CAN controller
}

void RobomasSender::SetRobomas(Robomas* robomas) {
    _robomas = robomas;
}
void RobomasSender::Start() {
    _can.attach(callback(this, &RobomasSender::Read), CAN::RxIrq);
}
bool RobomasSender::Send() {
    uint8_t success = 0;
    for (uint8_t i=0; i<2; i++) {
        CANMessage msg;
        msg.id = (i == 0) ? 0x200 : 0x1FF;
        msg.len = 8;
        msg.format = CANStandard;
        int16_t buff[4];
        for (uint8_t j=0; j<4; j++) {
            _robomas[j].GetSendBuff(&buff[j]);
        }
        msg.data[0] = (buff[0] >> 8) & 0xFF;
        msg.data[1] = buff[0] & 0xFF;
        msg.data[2] = (buff[1] >> 8) & 0xFF;
        msg.data[3] = buff[1] & 0xFF;
        msg.data[4] = (buff[2] >> 8) & 0xFF;
        msg.data[5] = buff[2] & 0xFF;
        msg.data[6] = (buff[3] >> 8) & 0xFF;
        msg.data[7] = buff[3] & 0xFF;
        success += _can.write(msg);
    }
    return (success == 2) ? true : false;
}
void RobomasSender::Read() {
    for (uint8_t i=0; i<8; i++) {
        CANMessage msg;
        msg.id = _robomas[i].GetId();
        msg.len = 8;
        msg.format = CANStandard;
        int16_t buff[4] = {0, 0, 0, 0};
        if (_can.read(msg)) {
            buff[0] = (msg.data[0] << 8) | msg.data[1];
            buff[1] = (msg.data[2] << 8) | msg.data[3];
            buff[2] = (msg.data[4] << 8) | msg.data[5];
            if (_robomas[i].GetMotorType() == MotorType::M3508) {
                buff[3] = (msg.data[6] << 8) | msg.data[7];
            }
            _robomas[i].SetReadData(buff);
        }
    }
}
uint8_t RobomasSender::GetWriteError() {
    return _can.tderror();
}
uint8_t RobomasSender::GetReadError() {
    return _can.rderror();
}
bool RobomasSender::GetBusOn() {
    return ((_can.rderror() >= 248) | (_can.tderror() >= 248)) ? false : true;
}
void RobomasSender::CanReset() {
    _can.reset();
}

// Robomas class implementation
Robomas::Robomas(MotorType type, uint8_t motor_num) {
    _type = type;
    _feedback_id = motor_num + 0x200;
    _motor_num = motor_num;
    _max_torque_limit = (type == MotorType::M2006) ? M2006_MAX_TORQUE : M3508_MAX_TORQUE;
    _torque_limit = _max_torque_limit;
    Init();
}

Robomas::~Robomas() = default;

// base functionality
void Robomas::GetSendBuff(int16_t* data) {
    *data = send_buff;
}
void Robomas::SetReadData(int16_t* data) {
    memcpy(read_data, data, sizeof(read_data));
}

// start/stop functionality
void Robomas::Init() {
    send_buff = 0;
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = 0;
    }
}

// set configure
void Robomas::SetId(uint16_t id) {
    _feedback_id = (id <= 8) ? id + 0x200 : id;
}
void Robomas::SetMotorNum(uint8_t number) {
    _motor_num = number;
}
void Robomas::SetMotorType(MotorType type)  {
    _type = type;
}
void Robomas::SetTorqueLimit(int16_t limit) {
    if (limit <= _max_torque_limit) {
        _torque_limit = limit;
    } else {
        _torque_limit = _max_torque_limit;
        printf("Robomas::SetTorqueLimit: 'Torque limit exceeds maximum limit. Setting to maximum.'\n");
    }
}

// get configure
uint16_t Robomas::GetId() const {
    return _feedback_id;
}
uint8_t Robomas::GetMotorNum() const {
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
    send_buff = (torque <= _torque_limit) ? torque : _torque_limit;
}
void Robomas::SetSpeed(uint8_t speed) {
    // PIDを実装
}
void Robomas::SetPosition(int16_t position) {
    // PIDを実装
}
void Robomas::SetBrake() {
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
