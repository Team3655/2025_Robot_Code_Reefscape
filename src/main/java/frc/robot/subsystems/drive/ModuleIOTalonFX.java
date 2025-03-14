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
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

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

  // private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;
  private final String canivoreName = Constants.CANIVORE_NAME;

  public ModuleIOTalonFX(int index) {

    Rotation2d flEncoderOffset = (Constants.currentRobot == RobotType.COMPBOT)
        ? DriveConstants.COMPBOT_FRONT_LEFT_ENCODER_OFFSET
        : DriveConstants.PROTOBOT_FRONT_LEFT_ENCODER_OFFSET;

    Rotation2d frEncoderOffset = (Constants.currentRobot == RobotType.COMPBOT)
        ? DriveConstants.COMPBOT_FRONT_RIGHT_ENCODER_OFFSET
        : DriveConstants.PROTOBOT_FRONT_RIGHT_ENCODER_OFFSET;

    Rotation2d blEncoderOffset = (Constants.currentRobot == RobotType.COMPBOT)
        ? DriveConstants.COMPBOT_BACK_LEFT_ENCODER_OFFSET
        : DriveConstants.PROTOBOT_BACK_LEFT_ENCODER_OFFSET;

    Rotation2d brEncoderOffset = (Constants.currentRobot == RobotType.COMPBOT)
        ? DriveConstants.COMPBOT_BACK_RIGHT_ENCODER_OFFSET
        : DriveConstants.PROTOBOT_BACK_RIGHT_ENCODER_OFFSET;

    switch (index) {
      // Front left
      case 0:
        driveTalon = new TalonFX(1, canivoreName);
        turnTalon = new TalonFX(2, canivoreName);
        cancoder = new CANcoder(3, canivoreName);
        absoluteEncoderOffset = flEncoderOffset;
        break;
      // Front right
      case 1:
        driveTalon = new TalonFX(4, canivoreName);
        turnTalon = new TalonFX(5, canivoreName);
        cancoder = new CANcoder(6, canivoreName);
        absoluteEncoderOffset = frEncoderOffset;
        break;
      // Back left
      case 2:
        driveTalon = new TalonFX(7, canivoreName);
        turnTalon = new TalonFX(8, canivoreName);
        cancoder = new CANcoder(9, canivoreName);
        absoluteEncoderOffset = blEncoderOffset;
        break;
      // Back right
      case 3:
        driveTalon = new TalonFX(10, canivoreName);
        turnTalon = new TalonFX(11, canivoreName);
        cancoder = new CANcoder(12, canivoreName);
        absoluteEncoderOffset = brEncoderOffset;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }
    final double DRIVE_GEAR_RATIO = (Constants.currentRobot == RobotType.COMPBOT)
        ? DriveConstants.COMPBOT_DRIVE_GEAR_RATIO
        : DriveConstants.PROTOBOT_DRIVE_GEAR_RATIO;

    var driveConfig = new TalonFXConfiguration();

    // Configure drive motor to use integrated encoder for velocity feedback
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Feedback.RotorToSensorRatio = 1.0;
    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    driveConfig.Slot0.kP = DriveConstants.KP_DRIVE;
    driveConfig.Slot0.kV = DriveConstants.KV_DRIVE;
    driveConfig.Slot0.kS = DriveConstants.KS_DRIVE;

    driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveTalon.getConfigurator().apply(driveConfig);

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

    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
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
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    // inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnPositionError = Rotation2d.fromRotations(turnTalon.getClosedLoopError().getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    // inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value))
        .toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromRotations(value))
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

  /*
   * @param Radians per second of the drive wheel
   * 
   */
  @Override
  public void setDriveVelocity(double radsPerSecond) {
    double rotationsPerSecond = Units.radiansToRotations(radsPerSecond);

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
}