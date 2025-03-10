// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final IntakeIO intakeIO;

  private double intakeVoltage;

  public IntakeSubsystem(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    Logger.processInputs("Inputs/Intake", inputs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeIO.updateInputs(inputs);

    intakeIO.setVoltage(intakeVoltage);
  }

  public void setIntakeVoltage(double voltage) {
    intakeVoltage = voltage;
  }

  public boolean checkCurrentSpike(double threshold) {
    return (inputs.intakeCurrentAmps[inputs.intakeCurrentAmps.length - 1] > threshold);
  }

}
