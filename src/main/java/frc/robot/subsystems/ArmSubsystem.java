package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_armMotor = new WPI_TalonFX(Constants.ArmConstants.kArmMotorPort);

  public ArmSubsystem() {
    // We don't want the motor to move
    m_armMotor.setNeutralMode(NeutralMode.Brake);
    m_armMotor.setInverted(false);
    m_armMotor.set(0.0);

    // Set motion magic parameters
    m_armMotor.configMotionProfileTrajectoryPeriod(Constants.ArmConstants.kArmTrajectoryPeriod);
    m_armMotor.configMotionSCurveStrength(Constants.ArmConstants.kArmSCurveStrength);
    m_armMotor.configMotionProfileTrajectoryInterpolationEnable(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void moveArm(double speed) {
    m_armMotor.selectProfileSlot(0, 0);
    m_armMotor.configMotionCruiseVelocity(Constants.ArmConstants.kArmCruiseVelocity, 0);
    m_armMotor.configMotionAcceleration(Constants.ArmConstants.kArmAcceleration, 0);
    m_armMotor.set(ControlMode.MotionMagic, speed);
  }

  public void stopArm() {
    m_armMotor.set(0);
  }

  public double getAngle() {
    return m_armMotor.getSelectedSensorPosition();
  }

}
