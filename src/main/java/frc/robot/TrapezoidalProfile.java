package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TrapezoidalProfile {
  // Maximum velocity in units per second
  private double maxVelocity;
  // Maximum acceleration in units per second^2
  private double maxAcceleration;
  // Distance to be covered
  private double distance;
  // Current time
  private double time;

  private WPI_TalonFX talonFX;

  public TrapezoidalProfile(double maxVelocity, double maxAcceleration, double distance, WPI_TalonFX talonFX) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.distance = distance;
    this.talonFX = talonFX;
    this.time = 0;
  }

  public void executeProfile() {
    double accelerationTime = maxVelocity / maxAcceleration;
    double accelerationDistance = 0.5 * maxAcceleration * accelerationTime * accelerationTime;
    double cruiseDistance = distance - 2 * accelerationDistance;
    double cruiseTime = cruiseDistance / maxVelocity;
    double decelerationTime = maxVelocity / maxAcceleration;

    while (time < accelerationTime) {
      double velocity = maxAcceleration * time;
      double position = 0.5 * maxAcceleration * time * time;
      talonFX.set(ControlMode.Position, position);
      talonFX.set(ControlMode.Velocity, velocity);
      time += 0.02; // Example value, update rate depends on your system
    }

    while (time < accelerationTime + cruiseTime) {
      double velocity = maxVelocity;
      double position = accelerationDistance + maxVelocity * (time - accelerationTime);
      talonFX.set(ControlMode.Position, position);
      talonFX.set(ControlMode.Velocity, velocity);
      time += 0.02; // Example value, update rate depends on your system
    }

    while (time < accelerationTime + cruiseTime + decelerationTime) {
      double velocity = maxVelocity - maxAcceleration * (time - accelerationTime - cruiseTime);
      double position = distance - 0.5 * maxAcceleration * (decelerationTime - (time - accelerationTime - cruiseTime))
          * (decelerationTime - (time - accelerationTime - cruiseTime));
      talonFX.set(ControlMode.Position, position);
      talonFX.set(ControlMode.Velocity, velocity);
      time += 0.02; // Example value, update rate depends on your system
    }
  }
}
