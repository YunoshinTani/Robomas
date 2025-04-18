/**
 * @file robomas.cpp
 * @brief Implementation file for Robomas class and RobomasSender class.
 * @details This file contains the implementation of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-19
 * @version 3.0.0
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

void RobomasSender::SetRobomas(Robomas* robomas) {
    _robomas = robomas;
}
void RobomasSender::ReadStart() {
    _can.attach(callback(this, &RobomasSender::Read), CAN::RxIrq);
}
bool RobomasSender::Send() {
    CANMessage msg;
    msg.id = 0x200;
    msg.len = 8;
    msg.format = CANStandard;
    int16_t buff[4];
    for (uint8_t j=0; j<4; j++) {
        buff[j] = _robomas[j].GetSendBuff();
    }
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
    int16_t buff[4] = {0, 0, 0, 0};
    CANMessage send_msg;
    if (_can.read(send_msg)) {
        buff[0] = (int16_t)((send_msg.data[0] << 8) | send_msg.data[1]);
        buff[1] = (int16_t)((send_msg.data[2] << 8) | send_msg.data[3]);
        buff[2] = (int16_t)((send_msg.data[4] << 8) | send_msg.data[5]);
        buff[3] = ((int16_t)send_msg.data[6]);
        _robomas[2].SetReadData(buff); // Set read data to the motor
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
int16_t Robomas::GetSendBuff() {
    return send_buff;
}
void Robomas::SetReadData(int16_t* data) {
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = data[i];
    }
}

// start/stop functionality
void Robomas::Init() {
    send_buff = 0;
    for (uint8_t i=0; i<4; i++) {
        read_data[i] = 0;
    }
    SetTorque(0); // Set torque to 0
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
uint16_t Robomas::GetPosition() {
    return read_data[0];
}
int16_t Robomas::GetVelocity() {
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
    printf("Pos:%4d,   ", GetPosition());    // Get motor position from c620[0]
    printf("Vel:%5d,   ", GetVelocity());    // Get motor position from c620[1]
    printf("Tor:%5d,   ", GetTorque());      // Get motor position from c620[2]
    printf("Tem:%2d,   ", GetTemperature()); // Get motor position from c620[3]
}

// Robomas class implementation
// RobomasSender class implementation
