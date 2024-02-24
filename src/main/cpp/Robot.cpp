// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#define SIZEOF_ARRAY(array_name) (sizeof(array_name) / sizeof(array_name[0]))

typedef struct
{
    move_step_t *steps;
    size_t total_steps;
} move_seq_t;

move_step_t mv_default[] =
{
  // delay
  // {0.0, 0.0, 5.0, 0.68, 0.0, 0, 0.0, 0.0},
  //{drive,turn angle,time,tilt(bool),bucket(bool),limelight(bool)}
  {-0.2, 0, 0.7, 0, 0, 0},  //forward
  {0, 0, 5.0, 0, 0, 0},
  {0, 90.0, 1.0, 0, 0, 0},  //turn toward amp
  {0, 0, 0.5, 0, 0, 1},     //aim with limelight
  {-0.2, 0, 0.9, 0, 0, 1},  //forward with limelight
  {0, 0, 0.2, 0, 0, 0},
  {0, 0, 1, 1, 1, 0},  //Dump it
  {0, 0, 0.2, 0, 0, 0},
  {0.2, 0, 0.4, 0, 0, 0}, //reverse
  {0, 0, 0.2, 0, 0, 0},
  {0, 5.0, 1.0, 0, 0, 0},  //turn right
  {0, 0, 0.2, 0, 0, 0},
  {-0.2, 0, 1.5, 0, 0, 0},  //forward
  {0, 0, 0.2, 0, 0, 0},
  // stop
  {0, 0, 0, 0, 0, 0},
};


move_step_t mv_auto_2[] =
{
  // delay
  {-0.2, 0, 2.5, 0, 0, 0},
  {0.0, 0, 0, 0, 0, 0},
0
};

move_step_t mv_auto_3[] =
{
  // delay
  // {0.0, 0.0, 5.0, 0.68, 0.0, 0, 0.0},
  {0.0, 0, 0, 0, 0, 0},
  {0.0, 0, 0, 0, 0, 0},
};

move_step_t none[] =
{
  {0.0, 0, 0, 0, 0, 0},
};

move_seq_t mv = {none, 0};

#if 1
static void clamp(double &value, const double ul, const double ll)
{
    if (value < ll)
  {
      value = ll;
  }
    else if (value > ul)
  {
      value = ul;
  }
}
#endif

#if 1
static void scale(double &value, const double deadband, const double ll, const double ul)
{
  bool positive = (value >= 0.0);

  if (!positive) value *= -1;

//printf("1: value=%5.2f\n", value);

  // adjust for deadband
  if (value > deadband) value -= deadband;
  // else if (value < -deadband) value += deadband;
  else value = 0;

//printf("2: value=%5.2f\n", value);

  // scale based output range / input range
  value *= ((ul - ll) / (1.0 - deadband));

//printf("3: value=%5.2f\n", value);

  // offset to minimum
  value += ll;
  // if (value > 0) value += ll;
  // else value -= ll;

  if (!positive) value *= -1;

//printf("4: value=%5.2f\n", value);
}
#endif



void Robot::RobotInit()
{
    m_timer.Reset();
    m_timer.Start();

    m_drive_rightMotor.RestoreFactoryDefaults();
    m_drive_rightFollower.RestoreFactoryDefaults();
    m_drive_leftMotor.RestoreFactoryDefaults();
    m_drive_leftFollower.RestoreFactoryDefaults();

    m_drive_rightMotor.SetInverted(true);
    m_drive_rightFollower.SetInverted(true);
    m_drive_leftMotor.SetInverted(false);
    m_drive_leftFollower.SetInverted(false);

    m_winch.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_hookSolenoid.Set(frc::DoubleSolenoid::kReverse);
    m_tiltSolenoid.Set(frc::DoubleSolenoid::kReverse);
    m_bucketSolenoid.Set(frc::DoubleSolenoid::kReverse);

    m_winch.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);

    frc::SmartDashboard::PutNumber("delay", 0);
    frc::SmartDashboard::PutNumber("fw_sp1", 0.2);
    frc::SmartDashboard::PutNumber("auto", 1);

    //distanceSensor.SetUpSource(9);
    //distanceSensor.SetSemiPeriodMode(true);
    //distanceSensor.SetSamplesToAverage(2);

    m_ledStrip.SetLength(kLength);
    m_ledStrip.SetData(m_ledBuffer);
    m_ledStrip.Start();

}

void Robot::AutonomousInit()
{
  m_delay = frc::SmartDashboard::GetNumber("delay", 0);

  turnPID.Reset();
  _pidgey.Reset();

    auto ally = frc::DriverStation::GetAlliance();
    if(ally.value() == frc::DriverStation::Alliance::kRed){
      frc::SmartDashboard::PutString("Alliance: ", "Red");
      blueAlliance = false;
    }
    else if(ally.value() == frc::DriverStation::Alliance::kBlue){
      frc::SmartDashboard::PutString("Alliance: ", "Blue");
      blueAlliance = true;
    }
    else{
      frc::SmartDashboard::PutString("Alliance: ", " ");
    }

  // determine the auto routine to use
  int auto_selection = frc::SmartDashboard::GetNumber("auto", 1);
  printf("auto = %d\n", auto_selection);

  if (auto_selection == 1)
  {
    // mv_default[0].t = m_delay;
    start_move(mv_default, SIZEOF_ARRAY(mv_default));
  }
  else if (auto_selection == 2)
  {
    // mv_auto_2[0].t = m_delay;
    start_move(mv_auto_2, SIZEOF_ARRAY(mv_auto_2));
  }
  else if (auto_selection == 3)
  {
    // mv_auto_3[0].t = m_delay;
    start_move(mv_auto_3, SIZEOF_ARRAY(mv_auto_3));
  }
  else
  {
    mv_default[0].t = m_delay;
    start_move(mv_default, SIZEOF_ARRAY(mv_default));
  }
  
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
}

void Robot::AutonomousPeriodic(){
  update_move();

  double y = 0.0;
  double z = 0.0;
  auto bucketValue = frc::DoubleSolenoid::kReverse;
  auto tiltValue = frc::DoubleSolenoid::kReverse;
  limeLight_drive = 0;

  
  
  if (step < mv.total_steps)
  {
    y = mv.steps[step].y;
    z = mv.steps[step].z;
    limeLight_drive = mv.steps[step].lime;
    bool bucketState = mv.steps[step].bucket;
    bool tiltState = mv.steps[step].tilt;
    
    if(bucketState){
      bucketValue = frc::DoubleSolenoid::kForward;
    }
    else{
      bucketValue = frc::DoubleSolenoid::kReverse;
    }

    if(tiltState){
      tiltValue = frc::DoubleSolenoid::kForward;
    }
    else{
      tiltValue = frc::DoubleSolenoid::kReverse;
    }
      
    if (y == 0.0)
  {
      // set sign for pure rotations
      // z *= rotation;
  }

  if(limeLight_drive){
    if(blueAlliance){
      z = limeLightSteer(z, 0); //use pipeline 1 ID6
      z *= -1;
    }
    else{
      z = limeLightSteer(z, 0); //use pipeline 2  ID5
      z *= -1;
    }
  }
  else if(z > 1 || z < -1 ){   //use the IMU to turn if the value is not 0.
    if(!blueAlliance){
      z *= -1;  //reverse the turn for red side of the field
    }
    z = IMUturn(z);
  }
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
  m_tiltSolenoid.Set(tiltValue);
  m_bucketSolenoid.Set(bucketValue);

  }
  drive(y, z);
}

void Robot::TeleopInit()
{
    auto ally = frc::DriverStation::GetAlliance();
    if(ally.value() == frc::DriverStation::Alliance::kRed){
      frc::SmartDashboard::PutString("Alliance: ", "Red");
      blueAlliance = false;
    }
    else if(ally.value() == frc::DriverStation::Alliance::kBlue){
      frc::SmartDashboard::PutString("Alliance: ", "Blue");
      blueAlliance = true;
    }
    else{
      frc::SmartDashboard::PutString("Alliance: ", " ");
    }
    
    const int kTimeoutMs = 30;
		_pidgey.SetFusedHeading(0.0, kTimeoutMs);

  m_timestamp = 0;
  move_active = false; // no active move

}

void Robot::TeleopPeriodic()
{
    auto bucketCylinder = frc::DoubleSolenoid::kReverse;
    auto tiltCylinder = frc::DoubleSolenoid::kReverse;
    auto hookCylinder = frc::DoubleSolenoid::kReverse;
    double winchMotor = 0;

    double y = m_joystick_driver.GetLeftY();
    double z = m_joystick_driver.GetRightX();

    if(1){
      scale(y, 0.15, 0.0, .7); // Set th deadband, lower limit and upper limit on drive
      scale(z, 0.15, 0.0, .20);	// and on steer
    }
    else{ //no carpet
      scale(y, 0.15, 0.0, .4); // Set th deadband, lower limit and upper limit on drive
      scale(z, 0.15, 0.0, .20);	// and on steer
    }



    if (m_joystick_operator.GetRawButton(2)){   //red button
      bucketCylinder = frc::DoubleSolenoid::kForward;
    }
    else if(m_joystick_operator.GetRawButton(1)){ //Green Button
      tiltCylinder = frc::DoubleSolenoid::kForward;
      for(int i = 0; i<22; i++){
        m_ledBuffer[i].SetHSV(60,255,100);
      }
      m_ledStrip.SetData(m_ledBuffer);
    }
    else if(m_joystick_operator.GetRawButton(4)){
      tiltCylinder = frc::DoubleSolenoid::kForward;
      bucketCylinder = frc::DoubleSolenoid::kForward;
    }
    else{
      tiltCylinder = frc::DoubleSolenoid::kReverse;
      bucketCylinder = frc::DoubleSolenoid::kReverse;
      for(int i = 0; i<22; i++){
        m_ledBuffer[i].SetHSV(1,255,100);
      }
      m_ledStrip.SetData(m_ledBuffer);
    }

    if (m_joystick_operator.GetLeftTriggerAxis() > 0.7 && m_joystick_operator.GetRawButton(3)){   // button
      hookCylinder = frc::DoubleSolenoid::kForward;
    }
    else{
      hookCylinder = frc::DoubleSolenoid::kReverse;
    }
  
    if(m_joystick_driver.GetRightTriggerAxis() > 0.7 && m_joystick_driver.GetLeftTriggerAxis() > 0.7){
      winchMotor = -0.75;
    }
    else if (m_joystick_driver.GetRightTriggerAxis() > 0.7 && m_joystick_driver.GetLeftBumper()){
      winchMotor = 0.45;
    }
    else{
      winchMotor = 0.0;
    }

    if(m_joystick_operator.GetRightTriggerAxis() > 0.5){
      hookCylinder = frc::DoubleSolenoid::kOff;
    }

    if (m_joystick_driver.GetRawButton(1)){   //green button for AMP
      if(blueAlliance){
        z = limeLightSteer(z, 0); //use pipeline 1 ID6
      }
      else{
        z = limeLightSteer(z, 0); //use pipeline 2  ID5
      }
    }

    else if (m_joystick_driver.GetRawButton(2)){   //red button SOURCE right sides
      if(blueAlliance){
        z = limeLightSteer(z, 1); //use pipeline 2  ID1
      }
      else{
        z = limeLightSteer(z, 2); //use pipeline 3  ID9
      }
    }

    else if (m_joystick_driver.GetRawButton(4)){   //blue button SOURCE left sides
      if(blueAlliance){
        z = limeLightSteer(z, 3); //use pipeline 4  ID2
      }
      else{
        z = limeLightSteer(z, 4); //use pipeline 5  ID10
      }
    }

    else{
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
    }

    m_bucketSolenoid.Set(bucketCylinder);
    m_hookSolenoid.Set(hookCylinder);
    m_tiltSolenoid.Set(tiltCylinder);
    m_winch.Set(winchMotor);

    drive(y, -z);

  // ultrasonic ranging
  //auto distance = distanceSensor.GetPeriod().value() * 1000;    // 1 uSec = 1 mm
  //printf("d = %5.2f\n", distance);

  //if(distance < 0.5 )
  //{
  //  set_leds(0, 255, 0);
  //}
}

void Robot::DisabledInit() 
{
  //m_drive_rm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  //m_drive_rf.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  //m_drive_lm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  //m_drive_lf.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit()
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
}
void Robot::TestPeriodic() {}

void Robot::set_leds(int r, int g, int b)
{
    for(unsigned int i = 0; i < m_ledBuffer.size(); i++)
    {
      m_ledBuffer[i].SetRGB(r, g, b);
    }
    m_ledStrip.SetData(m_ledBuffer);
}

void Robot::drive(double y, double z)
{
  double tmp_y = last_y;
  double tmp_z = last_z;

  // limit acceleration
  double dy = y - last_y;
  double dz = z - last_z;

  clamp(dy, accel_max, -accel_max);
  clamp(dz, accel_max, -accel_max);

  last_y = last_y + dy;
  last_z = last_z + dz;

  if (tmp_y != last_y || tmp_z != last_z)
  {
    // printf("y: %5.2f => %5.2f / z: %5.2f => %5.2f\n", y, last_y, z, last_z);
  }

  // if(limeLightDrive)
  // {
  //   last_z = z;
  // }

  m_drive.ArcadeDrive(-last_y, last_z, false);
}

void Robot::start_move(move_step_t *move, int count)
{
  mv.steps = move;
  mv.total_steps = count;

  auto tv = m_timer.Get();

  // t_step_end = m_timer.Get() + m_delay;
  //t_step_end = m_timer.Get() + mv.steps[0].t;
  t_step_end = tv.value() + mv.steps[0].t;
  step = 0;

  move_active = true;

  //printf("%d: y=%5.2f z=%5.2f t=%5.2f l=%5.2f f=%5.2f\n", step+1,
  //    mv.steps[0].y, mv.steps[0].z, mv.steps[0].t, mv.steps[0].tilt, mv.steps[0].bucket);
}


void Robot::update_move()
{
  if (!move_active) return;

  auto tv = m_timer.Get();

  printf("t=%5.2f end=%5.2f\n", tv.value(), t_step_end);
  if (step < mv.total_steps)
  {
      step_complete = tv.value() > t_step_end;

      if (step_complete)
      {
        step += 1;
          if (step < mv.total_steps)
          {
              t_step_end += mv.steps[step].t;

             // printf("%d: y=%5.2f z=%5.2f t=%5.2f l=%5.2f f=%5.2f, p=%d, i=%5.2f\n", step+1,
              //    mv.steps[step].y, mv.steps[step].z, mv.steps[step].t, mv.steps[step].tilt, mv.steps[step].bucket,
              //    mv.steps[step].pixy);
          }
          else
          {
              // sequence complete
            printf("%5.2f: move complete\n", tv.value());

            step = mv.total_steps;
            move_active = false;
          }
      }
  }
}


double Robot::limeLightSteer(double steering_adjust, int pipeVal){
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
	nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", pipeVal);
  double Kp = 0.05;  //turn down if oscillating 0.01
  double min_command = 0.03;
  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
  double tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);
  //frc::SmartDashboard::PutNumber("tx: ", tx);
  //frc::SmartDashboard::PutNumber("tv: ", tv);
  if(tv){
    if(tx > 0.8){
      steering_adjust = Kp*tx + min_command;
    }
    if(tx < -0.8){
      steering_adjust = Kp*tx - min_command;
    }
  }
  scale(steering_adjust, 0.03, 0.0, .20);  //scale the steering differently when using the limelight
  clamp(steering_adjust, 0.2, -0.2);
  return steering_adjust;
}

double Robot::IMUturn(double targetHeading){
  double turnVal = 0;

  turnPID.SetPID(0.2, 0.0, 0.05);

  if(turnPID.AtSetpoint()){
    //turnPID.Reset();
  }

  turnPID.SetTolerance(1, 1); //sets error tolerance and derivative tolerance
  double actualHeading = _pidgey.GetFusedHeading();
  
  turnVal = turnPID.Calculate(_pidgey.GetFusedHeading(), targetHeading);

   printf("%d: z=%5.2f, actualHeading=%5.2f, targetHeading=%5.2f, turnVal=%5.2f\n", mv.steps[step].z, actualHeading, targetHeading, turnVal);

  //scale(turnVal, 0.03, 0.0, .20);  //scale the steering differently when using the IMU
  clamp(turnVal, 0.2, -0.2);
  return turnVal;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
 