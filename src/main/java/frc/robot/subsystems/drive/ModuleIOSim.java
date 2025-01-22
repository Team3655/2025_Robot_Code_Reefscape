// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO.
 *
 * <p>
 * Uses two flywheel sims for the drive and turn motors, with the absolute
 * position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide
 * a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double TURN_JKG = 0.004;
  private static final double DRIVE_JKG = 0.025;

  public ModuleIOSim() {
    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, DRIVE_JKG, DriveConstants.DRIVE_GEAR_RATIO), DRIVE_GEARBOX);
    turnSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(TURN_GEARBOX, TURN_JKG, DriveConstants.TURN_GEAR_RATIO),
        TURN_GEARBOX);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] { Math.abs(driveSim.getCurrentDrawAmps()) };

    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] { Math.abs(turnSim.getCurrentDrawAmps()) };

    inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
    inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
    inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }
}
