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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and CANcoder
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware
 * configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward
 * motion on the drive motor will propel the robot forward) and copy the
 * reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private PositionVoltage positionVoltage = new PositionVoltage(0.0);

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;
  private final String canivoreName = DriveConstants.CANIVORE_NAME;

  public ModuleIOTalonFX(int index) {

    switch (index) {
      // Back right
      case 0:
        driveTalon = new TalonFX(0, canivoreName);
        turnTalon = new TalonFX(1, canivoreName);
        cancoder = new CANcoder(2, canivoreName);
        absoluteEncoderOffset = DriveConstants.BACK_RIGHT_ENCODER_OFFSET;
        break;
      // Back left
      case 1:
        driveTalon = new TalonFX(3, canivoreName);
        turnTalon = new TalonFX(4, canivoreName);
        cancoder = new CANcoder(5, canivoreName);
        absoluteEncoderOffset = DriveConstants.BACK_LEFT_ENCODER_OFFSET;
        break;
      // Front right
      case 2:
        driveTalon = new TalonFX(6, canivoreName);
        turnTalon = new TalonFX(7, canivoreName);
        cancoder = new CANcoder(8, canivoreName);
        absoluteEncoderOffset = DriveConstants.FRONT_RIGHT_ENCODER_OFFSET;
        break;
      // Front left
      case 3:
        driveTalon = new TalonFX(9, canivoreName);
        turnTalon = new TalonFX(10, canivoreName);
        cancoder = new CANcoder(11, canivoreName);
        absoluteEncoderOffset = DriveConstants.FRONT_LEFT_ENCODER_OFFSET;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();

    // Configure drive motor to use integrated encoder for velocity feedback
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Feedback.RotorToSensorRatio = 1.0;
    driveConfig.Slot0.kP = DriveConstants.KP_DRIVE;
    driveConfig.Slot0.kV = DriveConstants.KV_DRIVE;

    driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveTalon.getConfigurator().apply(driveConfig);
    // setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();

    // Configure turn motor to use CANCoder for position feedback
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    turnConfig.Feedback.RotorToSensorRatio = DriveConstants.TURN_GEAR_RATIO;
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Slot0.kP = DriveConstants.KP_TURN;

    turnConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.TURN_CURRENT_LIMIT;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnTalon.getConfigurator().apply(turnConfig, .25);
    setTurnBrakeMode(true);

    // Configure CANcoder
    var turnEncoderConfig = new CANcoderConfiguration();

    turnEncoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset.getRotations();
    turnEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    cancoder.getConfigurator().apply(turnEncoderConfig);

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] { driveCurrent.getValueAsDouble() };

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
        .minus(absoluteEncoderOffset);
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] { turnCurrent.getValueAsDouble() };

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
        .toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
        .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnPosition(Rotation2d setpoint) {
    turnTalon.setControl(positionVoltage.withPosition(setpoint.getRotations()));
  }

  @Override
  public void setDriveVelocity(double rotationsPerSecond) {
    driveTalon.setControl(new VelocityVoltage(rotationsPerSecond).withSlot(0));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = isTurnMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }

  @Override
  public Rotation2d getPositionError() {
    return new Rotation2d(
        Units.rotationsToRadians(
            turnTalon.getClosedLoopError().getValueAsDouble()));
  }

}
