// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private ClimberIO io;
  private static ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private static double climberVolts = 0.0;
  private static Rotation2d armPosition = new Rotation2d();
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
    io.setArmPosition(armPosition, ArmFeedforward);
  }

  public void updateClimberVoltage(double volts) {
    climberVolts = volts;
  }

  public void updateArmPosition(Rotation2d position) {
    armPosition = position;
  }
}
