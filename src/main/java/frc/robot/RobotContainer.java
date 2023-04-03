// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.cameraserver.CameraServer;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The driver's controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    private final CommandXboxController m_armController = new CommandXboxController(OIConstants.kArmControllerPort);

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem(m_armController);
    private final IntakeSubsystem m_intake = new IntakeSubsystem(m_armController);

    // Teleop Command
    private final TeleopCommand m_teleopCommand = new TeleopCommand(m_robotDrive, m_driverController);

    // Scott likes men
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Camera Stuff
        CameraServer.startAutomaticCapture();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(m_teleopCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link CommandXboxController}), and then
     * calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held

        // GET the SonarLint extension!
        m_driverController.rightBumper()
                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.3)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1.0)));
        m_driverController.leftBumper()
                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.6)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1.0)));
        m_armController.rightBumper()
                .onTrue(new InstantCommand(() -> m_arm.setArmPosition(-0.373)));
        m_armController.x()
                .onTrue(new InstantCommand(() -> m_arm.setArmPosition(-0.271)));
        m_armController.leftBumper()
                .onTrue(new InstantCommand(() -> m_arm.setArmPosition(0)));
        m_armController.y()
                .onTrue(new InstantCommand(m_intake::foward))
                .onFalse(new InstantCommand(m_intake::stopMotor));
        m_armController.b()
                .onTrue(new InstantCommand(m_intake::backwards))
                .onFalse(new InstantCommand(m_intake::stopMotor));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(Timer timer) {
        while (timer.get() < 6) {
            if (timer.get() < 2) {
                m_robotDrive.drive(-0.2, 0, 0, false);
                continue;
            }
            m_robotDrive.drive(0.2, 0, 0, false);
        }
        m_robotDrive.drive(0, 0, 0, false);

        return null;
    }
}
