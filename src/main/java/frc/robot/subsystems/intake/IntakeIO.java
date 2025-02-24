// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {

    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = new double[] {};
    public double intakeTemp = 0.0;

    public double vacuumAppliedVolts = 0.0;
    public double vacuumTemp = 0.0;

    public boolean hasCoral = false;

  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setVoltage(double volts) {
  }

  public default void setVacuumVoltage(double voltage) {
  }

  public default void setVacuumSolenoid(boolean state) {
  }
}
