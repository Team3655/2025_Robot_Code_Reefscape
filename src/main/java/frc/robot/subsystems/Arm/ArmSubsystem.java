// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConstants.ArmStates;

public class ArmSubsystem extends SubsystemBase {

  public record ArmPose(Rotation2d shoulderAngle, Rotation2d elbowAngle, Rotation2d wristAngle) {
  }

  private final ArmIO io;

  public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final ArmVisualizer currentVisualizer = new ArmVisualizer("current");
  private final ArmVisualizer setpointVisualizer = new ArmVisualizer("setpoint");

  private ArmPose setpoint;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ArmIO io) {
    this.io = io;
    updateSetpoint(ArmStates.START);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);

    Rotation2d shoulderSetPoint = setpoint.shoulderAngle;
    Rotation2d elbowSetPoint = setpoint.elbowAngle;
    // Rotation2d wristSetPoint = setpoint.wristAngle;

    double shoulderFFComponent = ArmConstants.SHOULDER_MASS_KG * Math.cos(inputs.shoulderPosition.getRadians());
    double elbowFFComponent = ArmConstants.ELBOW_MASS_KG * Math.cos(inputs.elbowPosition.getRadians());

    io.setShoulderPositionWithFeedForward(shoulderSetPoint, shoulderFFComponent);
    io.setElbowPositionWithFeedForward(elbowSetPoint, elbowFFComponent);

    setpointVisualizer.update(getSetPoint());
    currentVisualizer.update(getState());
  }

  public ArmPose getState() {
    return new ArmPose(
        inputs.shoulderPosition,
        inputs.elbowPosition,
        inputs.wristPosition);
  }

  public ArmPose getSetPoint() {
    return setpoint;
  }

  public void updateSetpoint(ArmPose pose){
    setpoint = pose;
  }

}
