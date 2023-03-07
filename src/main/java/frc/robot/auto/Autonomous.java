package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Autonomous {
  private final SendableChooser<Command> autonomousChooser;
  private final MecanumAutoBuilder builder;
  private final HashMap<String, Command> eventMap = new HashMap<>();
  private final DriveSubsystem driveSubsystem;

  public Autonomous(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // TODO: Figure out why this is not working correctly
    builder = new MecanumAutoBuilder(
        driveSubsystem::getPose,
        driveSubsystem::resetOdometry,
        DriveConstants.kDriveKinematics,
        // Position contollers
        new PIDConstants(0 /* AutoConstants.kPXController */, 0, 0),
        new PIDConstants(0 /* AutoConstants.kPYController */, 0, 0),
        AutoConstants.kMaxSpeedMetersPerSecond,
        speeds -> {
          double frontLeftSpeed = speeds.frontLeftMetersPerSecond;
          double frontRightSpeed = speeds.frontRightMetersPerSecond;
          double rearLeftSpeed = speeds.rearLeftMetersPerSecond;
          double rearRightSpeed = speeds.rearRightMetersPerSecond;

          double xSpeed = (frontLeftSpeed + rearLeftSpeed + frontRightSpeed + rearRightSpeed) / 4.0;
          double ySpeed = ((frontLeftSpeed + rearRightSpeed) + ((frontRightSpeed + rearLeftSpeed) * -1)) / 4.0;
          double rotSpeed = ((frontLeftSpeed + rearLeftSpeed) - (frontRightSpeed + rearRightSpeed))
              / (4.0 * Constants.DriveConstants.kTrackWidth);

          driveSubsystem.drive(xSpeed * -1, ySpeed, rotSpeed, false);
        },
        eventMap,
        driveSubsystem);
    autonomousChooser = new SendableChooser<Command>();
    autonomousChooser.setDefaultOption("no-op", new InstantCommand());
    autonomousChooser.addOption("Go Foward", goFoward());
    autonomousChooser.addOption("Go Left", goLeft());

    SmartDashboard.putData("Autonomous Chooser", autonomousChooser);
  }

  public Command getSelected() {
    return autonomousChooser.getSelected();
  }

  public Command goFoward() {
    return builder.fullAuto(
        PathPlanner.loadPath("Go Foward", AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  }

  public Command goLeft() {
    return builder.fullAuto(
        PathPlanner.loadPath("Go Left", AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  }
}
