/**
 * @file robomas.hpp
 * @brief Header file for Robomas class and RobomasSender class.
 * @details This file contains the definition of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-16
 * @version 0.0.1
 * 
 * @warning This code has not been tested yet.
 */

#ifndef ROBOMAS_HPP
#define ROBOMAS_HPP

#include "mbed.h"

enum class MotorType {
    M2006,
    M3508,
    GM6020,
};

class Robomas;

class RobomasSender {
public:
    RobomasSender(CAN& can);
    ~RobomasSender();
    void SetRobomas(Robomas* robomas);
    void Start();
    bool Send();
    void Read();

    uint8_t GetWriteError();
    uint8_t GetReadError();
    void CanReset();
private:
    CAN& _can;
    Robomas* _robomas;
};

class Robomas {
public:
    Robomas(RobomasSender& sender, MotorType type, uint8_t motor_num);
    ~Robomas();

    // base functionality
    void GetSendBuff(int16_t* data);
    void SetReadData(int16_t* data);

    // start operation
    void Init();

    // SetConfigure
    void SetMotorNum(uint8_t number);
    void SetTorqueLimit(int16_t limit);

    // GetConfigure
    int GetID() const;
    int GetMotorNum() const;
    MotorType GetMotorType() const;
    int16_t GetTorqueLimit() const;

    // main write
    void SetTorque(int16_t torque);
    void SetSpeed(uint8_t speed);
    void SetPosition(int16_t position);
    void StopMotor();

    // main read
    int16_t GetMotorTorque();
    int16_t GetMotorSpeed();
    int16_t GetMotorPosition();

protected:
    int send_buff[4] = {0, 0, 0, 0};
    int16_t read_data[4] = {0, 0, 0, 0};

private:
    RobomasSender& _sender;

    MotorType _type;
    int _id;
    uint8_t _motor_num;
    int16_t _torque_limit;

};

#endif // ROBOMAS_HPP