package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorPort,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  private final XboxController m_driverController;

  public IntakeSubsystem(XboxController driverController) {
    m_driverController = driverController;

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_driverController.getRightTriggerAxis() > 0.5) {
      intakeGamePiece();
    } else if (m_driverController.getLeftTriggerAxis() > 0.5) {
      outtakeGamePiece();
    } else {
      m_intakeMotor.set(0);
    }
  }

  public Command intakeGamePiece() {
    return run(
        () -> {
          m_intakeMotor.set(1);
        });
  }

  public Command outtakeGamePiece() {
    return run(
        () -> {
          m_intakeMotor.set(-1);
        });
  }
}
