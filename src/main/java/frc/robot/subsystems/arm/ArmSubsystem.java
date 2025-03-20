// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.ArmConstants.ArmEncoders;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class ArmSubsystem extends SubsystemBase {

  /**
   * A record to store the data for the arm position.
   * (x, y) marks the desired target of the elbow/wrist joint,
   * with origin at the rear of the robot frame where it would meet the ground.
   * <br>
   * </br>
   * xTarget, yTarget, wristangle
   */
  public record ArmPose(double xTarget, double yTarget, Rotation2d wristAngle) {
  }

  private final ArmIO io;

  private ArmKinematics armKinematics;

  public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  @SuppressWarnings("unused")
  private final ArmVisualizer currentVisualizer = new ArmVisualizer("Current");
  @SuppressWarnings("unused")
  private final ArmVisualizer setpointVisualizer = new ArmVisualizer("Setpoint");

  private static ArmPose setpoint;

  private Rotation2d[] targetAngles = new Rotation2d[2];

  public SysIdRoutine shoulderRoutine;

  public SysIdRoutineLog sysIdLog;

  public Notification encoderNotification;

  public Notification bumpNotification;
  public Notification invalidArmStateNotification;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ArmIO io) {
    this.io = io;

    armKinematics = new ArmKinematics(
        ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS,
        ArmConstants.H_TOWER_GROUND_HEIGHT_METERS,
        ArmConstants.H_TOWER_GROUND_HEIGHT_METERS,
        ArmConstants.SHOULDER_LENGTH_METERS,
        ArmConstants.ELBOW_LENGTH_METERS);

    Logger.processInputs("Inputs/Arm", inputs);
    updateSetpoint(ArmStates.START);

    encoderNotification = new Notification(
        NotificationLevel.WARNING,
        ArmConstants.activeEncoders.toString() + " Arm Encoders",
        "The arm is set to use " + ArmConstants.activeEncoders.toString() + " encoders");

    Elastic.sendNotification(encoderNotification);

    bumpNotification = new Notification(
        NotificationLevel.WARNING,
        "Bump Invalid",
        "Arm bump is pushing end effector outside of physical reach");

    invalidArmStateNotification = new Notification(
        NotificationLevel.ERROR,
        "INVALID ARM STATE",
        "x and y setpoints are resulting in invalid arm angles.  Check your calculations");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return shoulderRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return shoulderRoutine.dynamic(direction);
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {

    io.updateInputs(inputs);

    // Calculate arm angles using setpoint
    switch (Constants.currentMode) {
      // Use real encoder math for real robot
      case REAL:
        targetAngles[0] = armKinematics.getArmAngles(setpoint.xTarget, setpoint.yTarget,
            ArmConstants.activeEncoders)[0];
        targetAngles[1] = armKinematics.getArmAngles(setpoint.xTarget, setpoint.yTarget,
            ArmConstants.activeEncoders)[1];
        break;
      case SIM:
        // Use absolute encoder math for simulation
        targetAngles[0] = armKinematics.getArmAngles(setpoint.xTarget, setpoint.yTarget, ArmEncoders.ABSOLUTE)[0];
        targetAngles[1] = armKinematics.getArmAngles(setpoint.xTarget, setpoint.yTarget, ArmEncoders.ABSOLUTE)[1];
      case REPLAY:
        break;
      default:
        break;
    }

    // For readability
    Rotation2d shoulderSetPoint = targetAngles[0];
    Rotation2d elbowSetPoint = targetAngles[1];
    Rotation2d wristSetPoint = Rotation2d.fromDegrees(Rotation2d.fromDegrees(90).minus(elbowSetPoint).getDegrees() + setpoint.wristAngle.getDegrees());

    // Updates arm position
    io.setShoulderPosition(shoulderSetPoint);
    io.setElbowPosition(elbowSetPoint);
    io.setWristPosition(wristSetPoint);

    // Updates the current arm angles in ArmKinematics
    armKinematics.currentArmAngles[0] = inputs.shoulderPosition;
    armKinematics.currentArmAngles[1] = inputs.elbowPosition;

    // Update visualizers
    // setpointVisualizer.update(
    //     shoulderSetPoint.getDegrees() - 90,
    //     elbowSetPoint.getDegrees(),
    //     wristSetPoint.getDegrees() - 90);

    // currentVisualizer.update(
    //     inputs.shoulderPosition.getDegrees() - 90,
    //     inputs.elbowPosition.getDegrees(),
    //     inputs.wristPosition.getDegrees() - 90);

    // Logger.recordOutput("Arm/Mechanism2d/Setpoint", setpointVisualizer.arm);
    // Logger.recordOutput("Arm/Mechanism2d/Current", currentVisualizer.arm);

    RobotState.getInstance().updateArmState(
        inputs.shoulderPosition,
        inputs.elbowPosition,
        inputs.wristPosition,
        armKinematics.calculateForwardKinematics(inputs.shoulderPosition, inputs.elbowPosition)[0],
        armKinematics.calculateForwardKinematics(inputs.shoulderPosition, inputs.elbowPosition)[1]);

    // Live telemetry for arm setpoints and actual positions
    SmartDashboard.putNumber("ShoulderSetpoint", shoulderSetPoint.getDegrees());
    SmartDashboard.putNumber("ElbowSetpoint", elbowSetPoint.getDegrees());
    SmartDashboard.putNumber("WristSetpoint", wristSetPoint.getDegrees());
    SmartDashboard.putNumber("ShoulderDeg", inputs.shoulderPosition.getDegrees());
    SmartDashboard.putNumber("ElbowDeg", inputs.elbowPosition.getDegrees());
    //TODO: Unpack this
    SmartDashboard.putNumber("WristDeg", inputs.wristPosition.getDegrees() + inputs.elbowPosition.getDegrees() - 90);

  }

  /**
   * Gets the current state of the robot using forward kinematics
   * 
   * @return an ArmPose representing the current state
   */
  @AutoLogOutput(key = "Arm/Current")
  public ArmPose getState() {
    double[] armPosition = armKinematics.calculateForwardKinematics(inputs.shoulderPosition, inputs.elbowPosition);
    return new ArmPose(
        armPosition[0],
        armPosition[1],
        inputs.wristPosition);
  }

  /**
   * Gets the current setpoint
   * 
   * @return An ArmPose representing the setpoint
   */
  @AutoLogOutput(key = "Arm/Setpoint")
  public ArmPose getSetPoint() {
    return setpoint;
  }

  /**
   * Updates the setpoint to a new ArmPose
   * 
   * @param pose An ArmState to update the setpoint to
   */
  public void updateSetpoint(ArmPose pose) {

    if (armKinematics.isValidState(pose)) {
      setpoint = pose;
    } else {
      Elastic.sendNotification(invalidArmStateNotification);
    }
  }

  /**
   * Increment the wrist angle by degrees
   * 
   * @param degrees The degrees to increment by
   */
  public void jogWrist(double degrees) {
    setpoint = new ArmPose(setpoint.xTarget, setpoint.yTarget,
        setpoint.wristAngle.plus(Rotation2d.fromDegrees(degrees)));
  }

  public void bumpXArm(double inches) {
    ArmPose bumpSetpoint = new ArmPose(setpoint.xTarget + Units.inchesToMeters(inches),
        setpoint.yTarget,
        setpoint.wristAngle);

    if (armKinematics.isInsideArmReach(bumpSetpoint.xTarget, bumpSetpoint.yTarget)
        && armKinematics.isValidState(bumpSetpoint)) {
      setpoint = bumpSetpoint;
    } else {
      Elastic.sendNotification(bumpNotification);
    }
  }

  public void bumpYArm(double inches) {
    ArmPose bumpSetpoint = new ArmPose(setpoint.xTarget,
        setpoint.yTarget + Units.inchesToMeters(inches),
        setpoint.wristAngle);

    if (armKinematics.isInsideArmReach(bumpSetpoint.xTarget, bumpSetpoint.yTarget)) {
      setpoint = bumpSetpoint;
    } else {
      Elastic.sendNotification(bumpNotification);
    }
  }

  /**
   * Bumps the arm around a circle with a radius drawn from
   * the shoulder pivot point to the current setpoint
   * 
   * @param inches Arc length bumped. Negative values move the arm towards the
   *               back
   */
  public void bumpArmUsingArc(double arcLengthInches) {

    Translation2d radiusEndEffector = new Translation2d(
        setpoint.xTarget - ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS,
        setpoint.yTarget - ArmConstants.H_TOWER_GROUND_HEIGHT_METERS);

    double theta = Units.inchesToMeters(arcLengthInches) / radiusEndEffector.getNorm();

    // Translation of
    double bumpedX = (setpoint.xTarget - ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS) * Math.cos(theta)
        - (setpoint.yTarget - ArmConstants.H_TOWER_GROUND_HEIGHT_METERS) * Math.sin(theta)
        + ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS;

    double bumpedY = (setpoint.xTarget - ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS) * Math.sin(theta)
        + (setpoint.yTarget - ArmConstants.H_TOWER_GROUND_HEIGHT_METERS) * Math.cos(theta)
        + ArmConstants.H_TOWER_GROUND_HEIGHT_METERS;

    ArmPose bumpSetpoint = new ArmPose(
        bumpedX,
        bumpedY,
        setpoint.wristAngle);

    if (armKinematics.isInsideArmReach(bumpSetpoint.xTarget, bumpSetpoint.yTarget)
        && armKinematics.isValidState(bumpSetpoint)) {
      setpoint = bumpSetpoint;
    } else {
      Elastic.sendNotification(bumpNotification);
    }
  }
}
