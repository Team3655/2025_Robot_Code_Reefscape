// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final IntakeIO intakeIO;

  private double intakeVoltage;
  private double vacuumVoltage;

  public IntakeSubsystem(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeIO.updateInputs(inputs);

    intakeIO.setVoltage(intakeVoltage);
    intakeIO.setVacuumVoltage(vacuumVoltage);

    SmartDashboard.putBoolean("Has Coral", hasCoral());
    SmartDashboard.putBoolean("Current Spike", checkCurrentSpike(10));
  }

  public void setIntakeVoltage(double voltage) {
    if(voltage < 0 && inputs.hasCoral == true) {
      voltage = 0.0;
    }
    intakeVoltage = voltage;
  }

  public void setVacuumVoltage(double voltage) {
    vacuumVoltage = voltage;
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  public boolean checkCurrentSpike(double threshold) {
    return (inputs.intakeCurrentAmps[inputs.intakeCurrentAmps.length - 1] > threshold);
  }

}
