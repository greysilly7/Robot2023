// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.autonomous;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 4;

  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private XboxController m_stick;

  private double m_previousX = 0;
  private double m_speed = 0;

  private autonomous autonomous;

  @Override
  public void robotInit() {
    WPI_TalonFX frontLeft = new WPI_TalonFX(kFrontLeftChannel, "rio");
    WPI_TalonFX rearLeft = new WPI_TalonFX(kRearLeftChannel, "rio");
    WPI_TalonFX frontRight = new WPI_TalonFX(kFrontRightChannel, "rio");
    WPI_TalonFX rearRight = new WPI_TalonFX(kRearRightChannel, "rio");

    // Invert the right side motors
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new XboxController(kJoystickChannel);

    autonomous = new autonomous();
  }

  @Override
  public void autonomousInit() {
    // TODO Auto-generated method stub
    Command m_autonomousCommand = autonomous.getSelectedCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
    drive();
  }

  public void drive() {
    // Get the Joystick Inputs for the X, Y, and Z axis
    double x = m_stick.getLeftY() * -1;
    double y = m_stick.getRightX();
    double z = m_stick.getLeftX();

    // Add deadzones to the inputs
    final double kDeadzone = 0.15;
    final double kZDeadzone = 0.20;
    if (Math.abs(x) < kDeadzone) {
      x = 0.0;
    }
    if (Math.abs(y) < kDeadzone) {
      y = 0.0;
    }
    if (Math.abs(z) < kZDeadzone) {
      z = 0.0;
    }

    // Scale the inputs with trapezoidal motion
    final double kMaxSpeed = 0.50;
    final double kAcceleration = 0.015;
    final double kDeceleration = 0.015;

    double speed = kMaxSpeed;

    // Accelerate the robot if the input is increasing
    if (x > m_previousX) {
      speed = Math.min(speed, m_speed + kAcceleration);
    }
    // Decelerate the robot if the input is decreasing
    if (x < m_previousX) {
      speed = Math.max(speed, m_speed - kDeceleration);
    }

    m_previousX = x;
    m_speed = speed;

    // Scale the inputs
    x *= speed;
    y *= speed;
    z *= speed;

    // Drive the robot
    m_robotDrive.driveCartesian(x, y, z);
    m_robotDrive.feed();
  }
}
