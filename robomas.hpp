/**
 * @file robomas.hpp
 * @brief Header file for Robomas class and RobomasSender class.
 * @details This file contains the definition of the Robomas class, which represents a motor controller, and the RobomasSender class, which handles CAN communication with the motor controller.
 * The Robomas class provides methods for configuring and controlling the motor, while the RobomasSender class manages the sending and receiving of CAN messages.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 * @since 2025-04-16
 * @date 2025-04-29
 * @version 3.3.0
 */

#ifndef ROBOMAS_HPP
#define ROBOMAS_HPP

#include <mbed.h>

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

// Forward declaration of classes
class Robomas;
class RobomasSender;

/**
 * @brief Class for controlling a Robomas motor.
 * @details This class provides methods for configuring and controlling the motor, including setting torque, speed, and position.
 * @author Yunoshin Tani (taniyunoshin@gmail.com)
 */
class Robomas {
public:
    Robomas(MotorType type, uint8_t motor_num, float Kp = 0.0, float Ki = 0.0, float Kd = 0.0, std::chrono::milliseconds dt = 10ms);
    ~Robomas();

    // base functionality
    int16_t GetSendBuff();
    void SetReadData(int16_t* data);

    // start operation
    void ResetData();

    // SetConfigure
    void SetMotorNum(uint8_t number);
    void SetMotorType(MotorType type);
    void SetCurrentLimit(uint16_t limit);
    void SetTorqueLimit(uint16_t limit);
    void SetRpmLimit(uint16_t limit);

    // PID functionality
    void Pid(int16_t target, int16_t current, uint16_t max);
    void SetPidGain(float Kp, float Ki, float Kd, std::chrono::milliseconds dt);
    void ResetPid();
    void UpdatePid();

    // GetConfigure
    uint16_t  GetReadId() const;
    uint8_t   GetMotorNum() const;
    MotorType GetMotorType() const;
    uint16_t  GetCurrentLimit() const;
    uint16_t  GetTorqueLimit() const;
    uint16_t  GetRpmLimit() const;

    // main write
    void SetCurrent(int16_t current); // A
    void SetTorque(int16_t torque); // Nm
    void SetRpm(int16_t rpm); // rpm
    void SetPosition(int16_t target_position); // degree
    void SetBrake();

    // main read
    uint16_t GetPosition();
    int      GetTotalPosition();
    int16_t  GetRpm();
    int16_t  GetTorque();
    uint8_t  GetTemperature();

    void Debug();

protected:
    int16_t send_buff = 0; // current
    int16_t read_data[4] = {0, 0, 0, 0}; // Posi, RPM, Torque, Temp

private:
    const int MAX_MOTOR_NUM = 8; // Maximum number of motors

    struct M2006 {
        static constexpr float TORQUE_CONSTANT = 0.18; // Nm/A
        static constexpr int MAX_CURRENT = 10000; // mA
        static constexpr int MAX_RPM = 10000; // rpm
    };

    struct M3508{
        static constexpr float TORQUE_CONSTANT = 0.3; // Nm/A
        static constexpr int MAX_CURRENT = 10000; // mA
        static constexpr int MAX_RPM = 9500; // rpm
    };

    MotorType type_;
    uint16_t  feedback_id_;
    uint8_t   motor_num_;
    uint16_t  max_current_;
    uint16_t  max_rpm_;

    // PID control parameters
    float Kp_;
    float Ki_;
    float Kd_;

    float now_time_; // Get current time
    float prev_time_; // Previous time for PID calculation
    float sec_dt_; // Convert milliseconds to seconds

    int16_t  current_;
    int16_t  target_;
    uint16_t output_max_;
    float    error_;
    float    integral_;
    float    derivative_;
    float    prev_error_;
    int16_t  output_;
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
    void SetRobomas(Robomas* robomas, uint8_t robomas_num);
    void ReadStart();

    bool Send();
    void Read();
    void Debug();

    uint8_t GetWriteError();
    uint8_t GetReadError();
    void    CanReset();
private:
    CAN&     can_;
    Robomas* robomas_;
    int      can_frequency_;
    uint8_t  robomas_num_ = 1;
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

 
//  struct M2006 {
//     static constexpr uint8_t RATED_VOLTAGE = 24; // V
//     static constexpr float TORQUE_CONSTANT = 0.18; // Nm/A
//     static constexpr float SPEED_CONSTANT = 32.96; // rpm/V
//     static constexpr uint8_t SPEED_TORQUE_CONSTANT = 110;
//     static constexpr std::chrono::milliseconds MECHANICAL_TIME_CONSTANT = 53ms; // 52.78ms
//     static constexpr float PHASE_RESISTANCE = 461; // mOhm
//     static constexpr float PHASE_INDUCTANCE = 64.22; //μH
//     static constexpr uint8_t OPERATING_MAX_TEMPERATURE = 55; // C
//     static constexpr uint8_t NUMBER_OB_POLE_PAIRS = 7; // pairs
//     static constexpr uint8_t WEIGHT = 90; // g
//     static constexpr float REDUCTION_RATION = 36; // 36:1

//     static constexpr uint16_t ROTATIONAL_SPEED = 500; // rpm
//     static constexpr float CURRENT = 0.6; // A
//     static constexpr uint16_t RATED_ROTATIONAL_SPEED = 416; // rpm
//     static constexpr uint8_t RATED_TORQUE = 1; // Nm
//     static constexpr uint8_t RATED_CURRENT = 3; // A
//     static constexpr float MAX_EFFICIENCY = 0.66; // 66%
// };

// struct M3508{
//     static constexpr uint8_t RATED_VOLTAGE = 24; // V
//     static constexpr float TORQUE_CONSTANT = 0.3; // Nm/A
//     static constexpr float SPEED_CONSTANT = 24.48; // rpm/V
//     static constexpr uint8_t SPEED_TORQUE_CONSTANT = 72;
//     static constexpr std::chrono::milliseconds MECHANICAL_TIME_CONSTANT = 49ms; // ms
//     static constexpr float PHASE_RESISTANCE = 0.194; // Ohm
//     static constexpr float PHASE_INDUCTANCE = 0x097; //mH
//     static constexpr uint8_t OPERATING_MAX_TEMPERATURE = 50; // C
//     static constexpr uint8_t MAX_PERMISSIBLE_TEMPERATURE = 125; // C
//     static constexpr uint8_t NUMBER_OB_POLE_PAIRS = 7; // pairs
//     static constexpr uint16_t WEIGHT = 365; // g
//     static constexpr float REDUCTION_RATION = 3591/187; // 19.2:1

//     static constexpr uint16_t ROTATIONAL_SPEED = 482; // rpm
//     static constexpr float CURRENT = 0.78; // A
//     static constexpr uint16_t RATED_ROTATIONAL_SPEED = 469; // rpm
//     static constexpr uint8_t RATED_TORQUE = 3; // Nm
//     static constexpr uint8_t RATED_CURRENT = 10; // A
//     static constexpr float MAX_EFFICIENCY = 0.70; // 70%
//     static constexpr float STALL_TORQUE = 4.5; // Nm
//     static constexpr float STALL_CURRENT = 2.5; // A
// };