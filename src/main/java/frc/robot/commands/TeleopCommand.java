package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends CommandBase {
  private final CommandXboxController m_driverController;
  private final DriveSubsystem m_robotDrive;
  private final double kMaxSpeed;
  private final double kMaxAcceleration;

  private double m_prevTimeX;

  private final TrapezoidProfile.Constraints m_constraints;

  public TeleopCommand(DriveSubsystem robotDrive, CommandXboxController driverController) {
    m_robotDrive = robotDrive;
    m_driverController = driverController;

    // Default values
    kMaxSpeed = 9.5;
    kMaxAcceleration = 3.8;

    m_prevTimeX = 0.0;

    m_constraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration);

    addRequirements(m_robotDrive);
  }

  @Override
  public void execute() {
    double x = MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadzone);
    double y = MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDeadzone);
    double z = MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDeadzone);

    TrapezoidProfile profileX = new TrapezoidProfile(
        m_constraints,
        new TrapezoidProfile.State(m_robotDrive.getPose().getTranslation().getX(), 0),
        new TrapezoidProfile.State(x, 0));

    TrapezoidProfile profileY = new TrapezoidProfile(
        m_constraints,
        new TrapezoidProfile.State(m_robotDrive.getPose().getTranslation().getY(), 0),
        new TrapezoidProfile.State(y, 0));

    // double turnRate = maxTurnRate * z;
    double scaledZ = z * 0.5;
    TrapezoidProfile profileZ = new TrapezoidProfile(
        m_constraints,
        new TrapezoidProfile.State(m_robotDrive.getTurnRate(), 0),
        new TrapezoidProfile.State(scaledZ, 0));

    double currentTime = Timer.getFPGATimestamp();
    double deltaTime = currentTime - m_prevTimeX;
    m_prevTimeX = currentTime;

    double nextX = profileX.calculate(deltaTime).position;
    double nextY = profileY.calculate(deltaTime).position;
    double nextZ = profileZ.calculate(deltaTime).position;

    if (profileX.isFinished(deltaTime) && profileY.isFinished(deltaTime) && profileZ.isFinished(deltaTime)) {
      m_robotDrive.drive(0, 0, 0, false);
    } else {
      m_robotDrive.drive(nextX, nextY, nextZ, false);
    }
  }
}
