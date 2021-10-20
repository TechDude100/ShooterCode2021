// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.jfr.Percentage;

/**
 * This is a
 *  demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  WPI_TalonFX m_topMotor;
  WPI_TalonFX m_bottomMotor;
  XboxController controller;
  double topMotorRPM;
  double bottomMotorRPM;


  @Override
  public void robotInit() {

    m_topMotor = new WPI_TalonFX(11);
    m_bottomMotor = new WPI_TalonFX(12);
    controller = new XboxController(0);
    SmartDashboard.putNumber("topMotorRPM", 0);
    SmartDashboard.putNumber("bottomMotorRPM", 0);

    m_topMotor.config_kP(0, .1);
    m_bottomMotor.config_kP(0, .1);


  }

  @Override
  public void robotPeriodic() {
    topMotorRPM = SmartDashboard.getNumber("topMotorRPM", 0);
    bottomMotorRPM = SmartDashboard.getNumber("bottomMotorRPM", 0);

  }
  @Override
  public void teleopPeriodic() {

    if(controller.getYButton() == true) {
      m_topMotor.set(ControlMode.Velocity, RPMToTickPer100Ms(topMotorRPM));
      m_bottomMotor.set(ControlMode.Velocity, RPMToTickPer100Ms(bottomMotorRPM));
    }

    if(controller.getYButton() == false) {
      m_topMotor.set(ControlMode.Velocity, 0);
      m_bottomMotor.set(ControlMode.Velocity, 0);
    }
  }

  public double RPMToTickPer100Ms(double RPM) {
      // ticks per 100 miliseconds equation
    return ((RPM*1024*4)/(60*100));
  }
}
