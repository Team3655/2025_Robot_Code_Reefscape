// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private ClimberIO io;
  private static ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private static double climberVolts = 0.0;
  public static double ArmFeedforward = 0.0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
    Logger.processInputs("Inputs/Climber", inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setClimberVoltage(climberVolts);

    SmartDashboard.putNumber("Climber Volts", inputs.climberAppliedVolts);
    SmartDashboard.putNumber("Climber Position", getClimberPosition().getDegrees());
  }

  public void updateClimberVoltage(double volts) {
    climberVolts = volts;
  }

  public void driveClimber(double volts) {
    io.driveClimber(volts);
  }

  public Rotation2d getClimberPosition() {
    return inputs.climberPosition;
  }


}
