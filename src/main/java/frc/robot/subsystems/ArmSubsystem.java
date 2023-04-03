package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_armMotor = new WPI_TalonSRX(Constants.ArmConstants.kArmMotorPort);
  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);
  private final PIDController pidController = new PIDController(0.5, /* 0.65 */0, 0);
  private final CommandXboxController m_armController;

  private boolean usePresets = false;

  public ArmSubsystem(CommandXboxController armController) {
    m_armMotor.configContinuousCurrentLimit(10);
    m_armMotor.configPeakCurrentLimit(15);
    m_armMotor.enableCurrentLimit(true);
    m_armMotor.setNeutralMode(NeutralMode.Coast);
    m_armMotor.setInverted(false);
    m_encoder.reset();
    pidController.setSetpoint(0);
    SmartDashboard.putData(m_encoder);
    m_armController = armController;
  }

  // if(urmom)
  // and(urmom)
  @Override
  public void periodic() {
    double joystickPos = MathUtil.applyDeadband(m_armController.getLeftY(), 0.20);
    update(joystickPos);
  }

  public void setArmPosition(double position) {
    pidController.setSetpoint(position);
    usePresets = true;
  }

  public void update(double joystickInput) {
    if (usePresets) {
      m_armMotor.set(ControlMode.PercentOutput, pidController.calculate(m_encoder.getDistance()));
      if (Math.abs(m_encoder.getDistance() - pidController.getSetpoint()) <= 0.09)
        usePresets = false;
    } else {
      if (joystickInput != 0) {
        joystickInput = Math.copySign(joystickInput * joystickInput, joystickInput);
        double pidOutput = pidController.calculate(joystickInput);
        // double pidOutput = m_armController.getLeftY();
        m_armMotor.set(ControlMode.PercentOutput, Math.copySign(pidOutput, joystickInput));
      } else
        stopArm();
    }
  }

  public void stopArm() {
    m_armMotor.set(ControlMode.PercentOutput, 0);
  }

  // yo momma
  // hey guys, did you know in terms, there are forums
}
