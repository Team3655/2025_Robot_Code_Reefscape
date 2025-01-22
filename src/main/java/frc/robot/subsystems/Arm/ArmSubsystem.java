// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

public class ArmSubsystem extends SubsystemBase {

  // (x, y) marks the desired target of the elbow/wrist joint
  // with origin at the rear of the robot frame where it would meet the ground
  public record ArmPose(double xTarget, double yTarget, Rotation2d wristAngle) {
  }

  private final ArmIO io;

  public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final ArmVisualizer currentVisualizer = new ArmVisualizer("current");
  private final ArmVisualizer setpointVisualizer = new ArmVisualizer("setpoint");

  private static ArmPose setpoint;

  private static Rotation2d shoulderAngle = Rotation2d.fromRadians(0.0);
  private static Rotation2d elbowAngle = Rotation2d.fromRadians(0.0);

  private static Rotation2d elbowPositionToHorizontal = Rotation2d.fromRadians(0.0);
  private static Rotation2d shoulderPositionToHorizontal = Rotation2d.fromRadians(0.0);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ArmIO io) {
    this.io = io;
    updateSetpoint(ArmStates.START);
    DriverStation.reportWarning(
        "ARM IS SET TO USE " + ArmConstants.activeEncoders.toString() + " ENCODERS.  IS THIS CORRECT?", false);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);

    Rotation2d shoulderSetPoint = shoulderAngle;
    Rotation2d elbowSetPoint = elbowAngle;
    Rotation2d wristSetPoint = setpoint.wristAngle;

    // Shoulder segment measures zero from -y, use sin() for FF
    // Elbow segment measures zero from +x, use cos() for FF
    // All angles for FF MUST USE RELATIVE ANGLES
    // TODO: Check math on these
    double shoulderFFComponent = ArmConstants.SHOULDER_MASS_KG * Math.sin(shoulderPositionToHorizontal.getRadians())
        + ArmConstants.ELBOW_MASS_KG * Math.cos(elbowPositionToHorizontal.getRadians())
        + ArmConstants.WRIST_MASS_KG * Math.cos(inputs.wristPosition.getRadians());

    double elbowFFComponent = ArmConstants.ELBOW_MASS_KG * Math.cos(inputs.elbowPosition.getRadians());
    double wristFFComponent = 1;

    calculateTargetAngles();

    io.setWristPositionWithFeedForward(setpoint.wristAngle, wristFFComponent);
    io.setShoulderPositionWithFeedForward(shoulderSetPoint, shoulderFFComponent);
    io.setElbowPositionWithFeedForward(elbowSetPoint, elbowFFComponent);

    // Actual shoulder zero is pointed straight down. Subtract 180 to match
    // visualizer to reality
    setpointVisualizer.update(shoulderSetPoint.getDegrees() - 180, elbowSetPoint.getDegrees(),
        wristSetPoint.getDegrees());
    currentVisualizer.update(inputs.shoulderPosition.getDegrees(), inputs.elbowPosition.getDegrees(),
        inputs.wristPosition.getDegrees());

    SmartDashboard.putNumber("Shoulder Setpoint", shoulderSetPoint.getDegrees());
    SmartDashboard.putNumber("Elbow Setpoint", elbowSetPoint.getDegrees());
    SmartDashboard.putNumber("Wrist Degrees", inputs.wristPosition.getDegrees());
  }

  public ArmPose getState() {
    return new ArmPose(
        setpoint.xTarget,
        setpoint.yTarget,
        inputs.wristPosition);
  }

  public ArmPose getSetPoint() {
    return setpoint;
  }

  public void updateSetpoint(ArmPose pose) {
    setpoint = pose;
  }

  public static void calculateTargetAngles() {

    // OnShape:
    // https://cad.onshape.com/documents/0eb11a58606ee3c3dda8aa0d/w/d1c684d1c568543878764fb7/e/4c2980432bfd825f337a321f?renderMode=0&uiState=678c52c70a7cb65a2aa773cc
    double L4 = 0.0;
    double L5 = 0.0;
    double L6 = 0.0;

    Rotation2d theta1 = Rotation2d.fromRadians(0.0);
    Rotation2d relativeTheta2 = Rotation2d.fromRadians(0.0);
    Rotation2d absoluteTheta2 = Rotation2d.fromRadians(0.0);
    Rotation2d theta3 = Rotation2d.fromRadians(0.0);
    Rotation2d thetaL4 = Rotation2d.fromRadians(0.0);

    L4 = Math.sqrt(
        (Math.pow(setpoint.xTarget - ArmConstants.D_ARM_HORIZTONAL_OFFSET_METERS, 2)
            + Math.pow(setpoint.yTarget - ArmConstants.H_TOWER_GROUND_HEIGHT_METERS, 2)));

    L6 = Math.sqrt(
        Math.pow(setpoint.xTarget - ArmConstants.D_ARM_HORIZTONAL_OFFSET_METERS, 2) + Math.pow(
            setpoint.yTarget - ArmConstants.H_TOWER_GROUND_HEIGHT_METERS + ArmConstants.TOWER_CHASSIS_HEIGHT_METERS,
            2));

    theta3 = new Rotation2d(
        Math.acos(
            (Math.pow(L4, 2) + Math.pow(ArmConstants.SHOULDER_LENGTH_METERS, 2)
                - Math.pow(ArmConstants.ELBOW_LENGTH_METERS, 2)) /
                (2 * L4 * ArmConstants.SHOULDER_LENGTH_METERS)));

    // Robot moves arm behind itself
    if (setpoint.xTarget > ArmConstants.D_ARM_HORIZTONAL_OFFSET_METERS
        || setpoint.xTarget == ArmConstants.D_ARM_HORIZTONAL_OFFSET_METERS) {
      theta1 = new Rotation2d(
          Math.acos(
              (Math.pow(L4, 2) + Math.pow(ArmConstants.TOWER_CHASSIS_HEIGHT_METERS, 2) - Math.pow(L6, 2)) /
                  (2 * L4 * ArmConstants.TOWER_CHASSIS_HEIGHT_METERS))
              - theta3.getRadians());
    } else {
      theta1 = new Rotation2d(
          2 * Math.PI - theta3.getRadians() - Math.acos(
              (Math.pow(L4, 2) + Math.pow(ArmConstants.TOWER_CHASSIS_HEIGHT_METERS, 2) - Math.pow(L6, 2)) /
                  (2 * L4 * ArmConstants.TOWER_CHASSIS_HEIGHT_METERS)));
    }

    L5 = Math.sqrt(
        Math.pow(ArmConstants.TOWER_CHASSIS_HEIGHT_METERS, 2) + Math.pow(ArmConstants.SHOULDER_LENGTH_METERS, 2)
            - (2 * ArmConstants.TOWER_CHASSIS_HEIGHT_METERS * ArmConstants.SHOULDER_LENGTH_METERS)
                * Math.cos(theta1.getRadians()));

    thetaL4 = new Rotation2d(Math.acos(
        (Math.pow(ArmConstants.SHOULDER_LENGTH_METERS, 2) + Math.pow(ArmConstants.ELBOW_LENGTH_METERS, 2)
            - Math.pow(L4, 2)) /
            (2 * ArmConstants.SHOULDER_LENGTH_METERS * ArmConstants.ELBOW_LENGTH_METERS)));

    // Relative
    relativeTheta2 = new Rotation2d((Math.PI / 2) + theta1.getRadians() - thetaL4.getRadians());

    // Absolute
    absoluteTheta2 = new Rotation2d(Math.PI - thetaL4.getRadians());

    try {
      validateState(theta1.getRadians(), L4, L5, L6, theta1.getRadians(), relativeTheta2.getRadians(),
          theta3.getRadians(), thetaL4.getRadians());
    } catch (InvalidArmState e) {
      System.out.println(e.getMessage());
      throw e;
    }

    switch (ArmConstants.activeEncoders) {
      case RELATIVE:
        elbowAngle = relativeTheta2;
        break;

      case ABSOLUTE:
        elbowAngle = absoluteTheta2;
        break;
    }

    elbowPositionToHorizontal = relativeTheta2;

    shoulderPositionToHorizontal = theta1;

    shoulderAngle = theta1;

  }

  public static void validateState(double theta, double L4, double L5, double L6, double theta1, double relativeTheta2,
      double theta3, double thetaL4) throws InvalidArmState {
    if (theta > Math.PI || theta < 0) {
      throw new InvalidArmState("ARM SEGMENT 2 CANNOT EXTEND PAST 180 DEG");
    }
    double[] values = new double[7];

    values[0] = L4;
    values[1] = L5;
    values[2] = L6;
    values[3] = theta1;
    values[4] = relativeTheta2;
    values[5] = theta3;
    values[6] = thetaL4;

    for (int i = 0; i < 7; i++) {
      if (!Double.isFinite(values[i])) {
        throw new InvalidArmState("ARM OUT OF BOUNDS - INVALID X AND Y");
      }
    }
  }

  public static class InvalidArmState extends RuntimeException {

    public InvalidArmState(String m) {
      super(m);
    }
  }

  public void jogWrist(double degrees) {
    setpoint = new ArmPose(setpoint.xTarget, setpoint.yTarget, Rotation2d.fromDegrees(degrees));
  }

}
