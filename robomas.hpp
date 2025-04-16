/**
 * @file robomas.hpp
 * @brief Header file for Robomas class and RobomasSender class.
 * @details This file contains the definition of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-17
 * @version 2.0.0
 * 
 * @warning This code has not been tested yet.
 */

#ifndef ROBOMAS_HPP
#define ROBOMAS_HPP

#include "mbed.h"

#define CAN_FREQUENCY 1000000 // CAN bus frequency (1 Mbps)
#define MAX_MOTOR_NUM 8 // Maximum number of motors
#define M2006_MAX_TORQUE 5000 // Maximum torque for M2006 motor
#define M3508_MAX_TORQUE 7000 // Maximum torque for M3508 motor

enum class MotorType {
    M2006,
    M3508,
    GM6020,
};

class Robomas; // Forward declaration
class RobomasSender; // Forward declaration

/**
 * @brief Class for controlling a Robomas motor.
 * @details This class provides methods for configuring and controlling the motor, including setting torque, speed, and position.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 */
class Robomas {
public:
    Robomas(MotorType type, uint8_t motor_num);
    ~Robomas();

    // base functionality
    void GetSendBuff(int16_t* data);
    void SetReadData(int16_t* data);

    // start operation
    void Init();

    // SetConfigure
    void SetId(uint16_t id);
    void SetMotorNum(uint8_t number);
    void SetMotorType(MotorType type);
    void SetTorqueLimit(int16_t limit);

    // GetConfigure
    uint16_t GetId() const;
    uint8_t GetMotorNum() const;
    MotorType GetMotorType() const;
    int16_t GetTorqueLimit() const;

    // main write
    void SetTorque(int16_t torque);
    void SetSpeed(uint8_t speed);
    void SetPosition(int16_t position);
    void SetBrake();

    // main read
    int16_t GetMotorTorque();
    int16_t GetMotorSpeed();
    int16_t GetMotorPosition();

protected:
    int16_t send_buff = 0;
    int16_t read_data[4] = {0, 0, 0, 0};

private:
    MotorType _type;
    uint16_t _feedback_id;
    uint8_t _motor_num;
    int16_t _torque_limit;
    int16_t _max_torque_limit;
};

/**
 * @brief Class for handling CAN communication with the Robomas motor controller.
 * @details This class manages the sending and receiving of CAN messages to and from the Robomas motor controller.
 * It provides methods for initializing communication, sending commands, and reading feedback.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 */
class RobomasSender {
public:
    RobomasSender(CAN& can);
    ~RobomasSender();
    void InitCan();
    void SetRobomas(Robomas* robomas);
    void Start();

    bool Send();
    void Read();

    uint8_t GetWriteError();
    uint8_t GetReadError();
    bool GetBusOn();
    void CanReset();
private:
    CAN& _can;
    Robomas* _robomas;
};

#endif // ROBOMAS_HPP