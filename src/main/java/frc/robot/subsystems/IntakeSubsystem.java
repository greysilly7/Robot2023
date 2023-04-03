package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorPort,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  private final CommandXboxController m_controller;
  private boolean isInverted = false;

  public IntakeSubsystem(CommandXboxController armController) {
    m_controller = armController;

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   if (m_controller.getLeftTriggerAxis() > 0.15) {
      isInverted = true;
    } else {
      isInverted = false;
    }
  }

  public void foward() {
    if (isInverted) {
      m_intakeMotor.set(-0.45);
    } else {
      m_intakeMotor.set(0.35);
    } 
  }

  public void backwards() {
    if (isInverted) {
      m_intakeMotor.set(0.35);
    } else {
      m_intakeMotor.set(-0.45);
    }
  }

  public void stopMotor() {
    m_intakeMotor.set(0);
  }
}
