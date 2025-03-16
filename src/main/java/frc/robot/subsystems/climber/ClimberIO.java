// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public class ClimberIOInputs {

    public double climberVelocityRadPerSec = 0.0;
    public double climberAppliedVolts = 0.0;
    public double[] climberCurrentAmps = new double[] {};

  }

  public default void updateInputs(ClimberIOInputs inputs) {
  }

  public default void setClimberVoltage(double volts) {
  }

  public default void driveClimber(double volts) {
  }
}
