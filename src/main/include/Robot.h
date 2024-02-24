/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/I2C.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/DutyCycleEncoder.h>
#include <frc/DriverStation.h>
#include <frc/AddressableLED.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/PIDController.h>


#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>

typedef struct
{
    double y;
    double z;
    double t;
    bool tilt;
    bool bucket;
    bool lime;

} move_step_t;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void set_leds(int r, int g, int b);
  void drive(double y, double z);
  void start_move(move_step_t *move, int count);
  void update_move();

  void update_Feeder(double m_feed);

  double limeLightSteer(double SteerVal, int pipeVal);

  double IMUturn(double targetHeading);
  
  rev::CANSparkMax m_drive_rightMotor{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_leftMotor{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_rightFollower{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_leftFollower{3, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_winch{6, rev::CANSparkMax::MotorType::kBrushless};
  
  frc::DifferentialDrive m_drive{m_drive_leftMotor, m_drive_rightMotor};

  frc::DoubleSolenoid m_tiltSolenoid{frc::PneumaticsModuleType::CTREPCM, 4, 3};
  frc::DoubleSolenoid m_bucketSolenoid{frc::PneumaticsModuleType::CTREPCM, 5, 2};
  frc::DoubleSolenoid m_hookSolenoid{frc::PneumaticsModuleType::CTREPCM, 6, 1};

  WPI_PigeonIMU _pidgey{0};

  frc::XboxController m_joystick_driver{0};
  frc::XboxController m_joystick_operator{1};

  frc::Timer m_timer;
  
  double m_timestamp {0};

  double m_delay;

  bool limeLight_drive = false;

  size_t total_steps;
  size_t step;
  double t_step_end;

  bool move_active {0};
  bool step_complete;

  double last_z {0};
  double last_y {0};

  double m_direction {1};
  double accel_max {0.02};

  bool blueAlliance;

  frc::Counter distanceSensor{frc::Counter::Mode::kSemiperiod};

  static constexpr int kLength = 22; //how many LEDs in the strip
  frc::AddressableLED m_ledStrip{9}; //PWM port
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer; 

  //frc::DriverStation::Alliance m_alliance {frc::DriverStation::Alliance::kInvalid};

  double kP, kI, kD;
  
  frc::PIDController turnPID{kP, kI, kD};
    
  rev::SparkMaxRelativeEncoder m_encoder_rightMotor = m_drive_rightMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  
};