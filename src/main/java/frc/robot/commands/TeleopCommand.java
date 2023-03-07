package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends CommandBase {
  private final XboxController m_driverController;
  private final DriveSubsystem m_robotDrive;

  private final TrapezoidProfile.Constraints m_constraints;

  public TeleopCommand(DriveSubsystem robotDrive, XboxController driverController) {
    m_robotDrive = robotDrive;
    m_driverController = driverController;

    double kMaxSpeed = 2.0;
    double kMaxAcceleration = 0.5;

    m_constraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration);

    addRequirements(m_robotDrive);
  }

  @Override
  public void execute() {
    double x = m_driverController.getLeftY();
    double y = m_driverController.getLeftX();
    double z = m_driverController.getRightX();

    if (Math.abs(x) < OIConstants.kDeadzone) {
      x = 0.0;
    }
    if (Math.abs(y) < OIConstants.kDeadzone) {
      y = 0.0;
    }
    if (Math.abs(z) < OIConstants.kDeadzone) {
      z = 0.0;
    }

    // Create a trapezoidal profile for each axis
    var wheelSpeeds = m_robotDrive.getCurrentWheelSpeeds();
    double frontLeftSpeed = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
    double rearLeftSpeed = wheelSpeeds.rearLeftMetersPerSecond;
    double rearRightSpeed = wheelSpeeds.rearRightMetersPerSecond;

    double currentXVelocity = (frontLeftSpeed + rearLeftSpeed + frontRightSpeed + rearRightSpeed) / 4.0;
    double currentYVelocity = ((frontLeftSpeed + rearRightSpeed) + ((frontRightSpeed + rearLeftSpeed) * -1)) / 4.0;
    double currentZVelocity = m_robotDrive.getAngularVelocity();

    TrapezoidProfile profileX = new TrapezoidProfile(m_constraints, new TrapezoidProfile.State(currentXVelocity, 0),
        new TrapezoidProfile.State(x, 0));
    TrapezoidProfile profileY = new TrapezoidProfile(m_constraints, new TrapezoidProfile.State(currentYVelocity, 0),
        new TrapezoidProfile.State(-y, 0));
    TrapezoidProfile profileZ = new TrapezoidProfile(m_constraints, new TrapezoidProfile.State(currentZVelocity, 0),
        new TrapezoidProfile.State(z, 0));
    // Generate the next setpoints for each profile
    double nextX = profileX.calculate(m_robotDrive.getPose().getX()).position;
    double nextY = profileY.calculate(m_robotDrive.getPose().getY()).position;
    double nextZ = profileZ.calculate(m_robotDrive.getHeading()).position;

    // Set the next setpoints on the drive
    m_robotDrive.drive(nextX, nextY, nextZ, false);
  }
}
