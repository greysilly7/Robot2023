package frc.robot.autonomous;

// TODO: Find out why this complains, no actual probem though
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class autonomous {
  private SendableChooser<Command> autonomousChooser;

  public autonomous() {
    // TODO: Figure out how to actually do this
    autonomousChooser = new SendableChooser<Command>();
    autonomousChooser.setDefaultOption("no-op", new InstantCommand());

    SmartDashboard.putData("Autonomous Chooser", autonomousChooser);
  }

  public Command getSelectedCommand() {
    return autonomousChooser.getSelected();
  }
}
