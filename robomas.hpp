/**
 * @file robomas.hpp
 * @brief Header file for Robomas class and RobomasSender class.
 * @details This file contains the definition of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-19
 * @version 3.0.0
 */

#ifndef ROBOMAS_HPP
#define ROBOMAS_HPP

#include "mbed.h"

enum class MotorType {
    M2006 = 0,
    M3508 = 1,
};

enum class ControlType {
    None = 0,
    Acceleration = 1,
    Velocity  = 2,
    Position  = 3,
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
    int16_t GetSendBuff();
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
    uint16_t GetPosition();
    int16_t GetVelocity();
    int16_t GetTorque();
    uint8_t GetTemperature();
    void Debug();

protected:
    int16_t send_buff = 0;
    int16_t read_data[4] = {0, 0, 0, 0};
    // Posi, Velo, Torque, Temp

private:
    const int MAX_MOTOR_NUM = 8; // Maximum number of motors
    const int M2006_MAX_TORQUE = 5000; // Maximum torque for M2006 motor
    const int M3508_MAX_TORQUE = 7000; // Maximum torque for M3508 motor

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
    RobomasSender(CAN& can, int can_frequency);
    ~RobomasSender();
    void InitCan();
    void SetRobomas(Robomas* robomas);
    void ReadStart();

    bool Send();
    void Read();
    void Debug();

    uint8_t GetWriteError();
    uint8_t GetReadError();
    bool GetBusOn();
    void CanReset();
private:
    CAN& _can;
    Robomas* _robomas;
    int _can_frequency;
};

#endif // ROBOMAS_HPP

/**
 * メモ
 * Position : unsigned 8*2bit
 * Velocity : signed 8*2bit
 * Torque   : signed 8*2bit
 * Temperature : unsigned 8bit
 * 
 * read_msg.data[7] is Null
 */