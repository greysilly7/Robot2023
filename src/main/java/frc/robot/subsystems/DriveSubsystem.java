// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_frontLeft = new WPI_TalonFX(DriveConstants.kFrontLeftMotorPort);
  private final WPI_TalonFX m_rearLeft = new WPI_TalonFX(DriveConstants.kRearLeftMotorPort);
  private final WPI_TalonFX m_frontRight = new WPI_TalonFX(DriveConstants.kFrontRightMotorPort);
  private final WPI_TalonFX m_rearRight = new WPI_TalonFX(DriveConstants.kRearRightMotorPort);

  public final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new MecanumDriveWheelPositions());

  // Creates a new DriveSubsystem.
  public DriveSubsystem() {
    m_frontLeft.setNeutralMode(NeutralMode.Brake);
    m_rearLeft.setNeutralMode(NeutralMode.Brake);
    m_frontRight.setNeutralMode(NeutralMode.Brake);
    m_rearRight.setNeutralMode(NeutralMode.Brake);

    // Fancy Dancy Encoder Stuff
    m_frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
        DriveConstants.kEncoderTimeoutMS);
    m_frontLeft.setSensorPhase(true);
    m_frontLeft.setSelectedSensorPosition(0, 0, 10); // We need to invert one side of the drivetrain so that positive

    m_rearLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
        DriveConstants.kEncoderTimeoutMS);
    m_rearLeft.setSensorPhase(true);
    m_rearLeft.setSelectedSensorPosition(0, 0, 10); // We need to invert one side of the drivetrain so that positive

    m_frontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
        DriveConstants.kEncoderTimeoutMS);
    m_frontRight.setSensorPhase(false);
    m_frontRight.setSelectedSensorPosition(0, 0, DriveConstants.kEncoderTimeoutMS);

    m_rearRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
        DriveConstants.kEncoderTimeoutMS);
    m_rearRight.setSensorPhase(false);
    m_rearRight.setSelectedSensorPosition(0, 0, DriveConstants.kEncoderTimeoutMS);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontLeft.setInverted(true);
    m_rearLeft.setInverted(true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

  /*
   * public void setMotorSpeedsAuto(MecanumDriveWheelSpeeds speeds) {
   * double frontLeftSpeed = speeds.frontLeftMetersPerSecond;
   * double frontRightSpeed = speeds.frontRightMetersPerSecond;
   * double rearLeftSpeed = speeds.rearLeftMetersPerSecond;
   * double rearRightSpeed = speeds.rearRightMetersPerSecond;
   * 
   * // Scale the speeds based on the gear ratio
   * frontLeftSpeed /= DriveConstants.kGearRatio;
   * frontRightSpeed /= DriveConstants.kGearRatio;
   * rearLeftSpeed /= DriveConstants.kGearRatio;
   * rearRightSpeed /= DriveConstants.kGearRatio;
   * 
   * // Set the motor speeds
   * m_frontLeft.set(frontLeftSpeed);
   * m_rearLeft.set(rearLeftSpeed);
   * m_frontRight.set(frontRightSpeed);
   * m_rearRight.set(rearRightSpeed);
   * }
   */

  /** Sets the motors to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.setSelectedSensorPosition(0);
    m_rearLeft.setSelectedSensorPosition(0);
    m_frontRight.setSelectedSensorPosition(0);
    m_rearRight.setSelectedSensorPosition(0);
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    double frontLeftSpeed = m_frontLeft.getSelectedSensorVelocity() * 0.1 * DriveConstants.kEncoderMetersPerRotation
        / DriveConstants.kGearRatio;
    double frontRightSpeed = m_frontRight.getSelectedSensorVelocity() * 0.1 * DriveConstants.kEncoderMetersPerRotation
        / DriveConstants.kGearRatio;
    double rearLeftSpeed = m_rearLeft.getSelectedSensorVelocity() * 0.1 * DriveConstants.kEncoderMetersPerRotation
        / DriveConstants.kGearRatio;
    double rearRightSpeed = m_rearRight.getSelectedSensorVelocity() * 0.1 * DriveConstants.kEncoderMetersPerRotation
        / DriveConstants.kGearRatio;
    return new MecanumDriveWheelSpeeds(frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a
   *         MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    double frontLeftPosition = m_frontLeft.getSelectedSensorPosition()
        * (DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio);
    double frontRightPosition = m_frontRight.getSelectedSensorPosition()
        * (DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio);
    double rearLeftPosition = m_rearLeft.getSelectedSensorPosition()
        * (DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio);
    double rearRightPosition = m_rearRight.getSelectedSensorPosition()
        * (DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio);
    return new MecanumDriveWheelPositions(
        frontLeftPosition,
        frontRightPosition,
        rearLeftPosition,
        rearRightPosition);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public double getAngularVelocity() {
    MecanumDriveWheelSpeeds wheelSpeeds = getCurrentWheelSpeeds();
    double frontLeftSpeed = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
    double rearLeftSpeed = wheelSpeeds.rearLeftMetersPerSecond;
    double rearRightSpeed = wheelSpeeds.rearRightMetersPerSecond;

    double angularVel = ((frontLeftSpeed + rearLeftSpeed) - (frontRightSpeed + rearRightSpeed))
        / 4.0;

    return angularVel;
  }
}
