// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

public class ArmSubsystem extends SubsystemBase {

  // (x, y) marks the desired target of the elbow/wrist joint
  // with origin at the rear of the robot frame where it would meet the ground
  public record ArmPose(double xTarget, double yTarget, Rotation2d wristAngle) {}

  private final ArmIO io;

  private ArmKinematics armKinematics;

  public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final ArmVisualizer currentVisualizer = new ArmVisualizer("current");
  private final ArmVisualizer setpointVisualizer = new ArmVisualizer("setpoint");

  private static ArmPose setpoint;

  private Rotation2d[] targetAngles = new Rotation2d[2];

  public SysIdRoutine shoulderRoutine;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ArmIO io) {
    this.io = io;
    updateSetpoint(ArmStates.START);
    DriverStation.reportWarning(
        "ARM IS SET TO USE " + ArmConstants.activeEncoders.toString() + " ENCODERS. IS THIS CORRECT?", false);
        armKinematics = new ArmKinematics(
          ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS, 
          ArmConstants.H_TOWER_GROUND_HEIGHT_METERS, 
          ArmConstants.H_TOWER_GROUND_HEIGHT_METERS, 
          ArmConstants.SHOULDER_LENGTH_METERS, 
          ArmConstants.ELBOW_LENGTH_METERS);

    shoulderRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.25).per(Second),
            Volts.of(0.5),
            Seconds.of(5),
            (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),

        new SysIdRoutine.Mechanism(
            (voltage) -> io.setShoulderVoltage(voltage.in(Volts)), 
            null, 
            this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return shoulderRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return shoulderRoutine.dynamic(direction);
  }

  @Override
  /**
   * This method will be called once per scheduler run
   */
  public void periodic() {

    io.updateInputs(inputs);

    // Calculate arm angles using setpoint
    targetAngles[0] = armKinematics.getArmAngles(setpoint.xTarget, setpoint.yTarget)[0];
    targetAngles[1] = armKinematics.getArmAngles(setpoint.xTarget, setpoint.yTarget)[1];

    targetAngles[0] = Rotation2d.fromDegrees(0);
    targetAngles[1] = Rotation2d.fromDegrees(0);

    // For readability
    Rotation2d shoulderSetPoint = targetAngles[0];
    Rotation2d elbowSetPoint = targetAngles[1];
    Rotation2d wristSetPoint = setpoint.wristAngle;

    // io.setShoulderPosition(shoulderSetPoint);
    // io.setElbowPosition(elbowSetPoint);
    // io.setWristPosition(setpoint.wristAngle);

    // Update visualizers
    setpointVisualizer.update(shoulderSetPoint.getDegrees(), elbowSetPoint.getDegrees(), wristSetPoint.getDegrees());
    Logger.recordOutput("Arm/Mechanism2dSetpoint", setpointVisualizer.arm);

    currentVisualizer.update(inputs.shoulderPosition.getDegrees(), inputs.elbowPosition.getDegrees(), inputs.wristPosition.getDegrees());
    Logger.recordOutput("Arm/Mechanism2dCurrent" , currentVisualizer.arm);

    SmartDashboard.putNumber("Shoulder Setpoint", shoulderSetPoint.getDegrees());
    Logger.recordOutput("Arm/Shoulder/Setpoint", shoulderSetPoint.getDegrees());
    Logger.recordOutput("Arm/Shoulder/Degrees", inputs.shoulderPosition.getDegrees());
    
    SmartDashboard.putNumber("Elbow Setpoint", elbowSetPoint.getDegrees());
    Logger.recordOutput("Arm/Elbow/Setpoint", elbowSetPoint.getDegrees());
    Logger.recordOutput("Arm/Elbow/Degrees", inputs.elbowPosition.getDegrees());

    SmartDashboard.putNumber("Wrist Degrees", inputs.wristPosition.getDegrees());
    Logger.recordOutput("Arm/Wrist/Setpoint", wristSetPoint.getDegrees());
    Logger.recordOutput("Arm/Wrist/Degrees", inputs.wristPosition.getDegrees());
  }

  /**
   * 
   * @return The current state of the arm
   */
  public ArmPose getState() {
    double[] armPosition = armKinematics.calculateForwardKinematics(inputs.shoulderPosition, inputs.elbowPosition);
    
    return new ArmPose(
        armPosition[0],
        armPosition[1],
        inputs.wristPosition);
  }

  /** 
   * @return the current setpoint
   */
  public ArmPose getSetPoint() {
    return setpoint;
  }

  /** 
   * Updates the setpoint to a new ArmPose
   */
  public void updateSetpoint(ArmPose pose) {
    setpoint = pose;
  }

  public void jogWrist(double degrees) {
    setpoint = new ArmPose(setpoint.xTarget, setpoint.yTarget, setpoint.wristAngle.plus(Rotation2d.fromDegrees(degrees)));
  }

}
