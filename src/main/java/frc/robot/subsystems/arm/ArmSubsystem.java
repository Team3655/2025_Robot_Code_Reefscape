// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.ArmConstants.ArmEncoders;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

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

  private final ArmVisualizer currentVisualizer = new ArmVisualizer("Current");
  private final ArmVisualizer setpointVisualizer = new ArmVisualizer("Setpoint");

  private static ArmPose setpoint;

  private Rotation2d[] targetAngles = new Rotation2d[2];

  public SysIdRoutine shoulderRoutine;

  public SysIdRoutineLog sysIdLog;

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
            (state) -> Logger.recordOutput("Arm/SysIdState", 
            state.toString())),

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
    Rotation2d wristSetPoint = setpoint.wristAngle;

    // Updates arm position
    io.setShoulderPosition(shoulderSetPoint); 
    io.setElbowPosition(elbowSetPoint);
    io.setWristPosition(wristSetPoint);

    // Updates the current arm angles in ArmKinematics
    armKinematics.currentArmAngles[0] = inputs.shoulderPosition;
    armKinematics.currentArmAngles[1] = inputs.elbowPosition;

    // Update visualizers
    setpointVisualizer.update(
        shoulderSetPoint.getDegrees() - 90,
        elbowSetPoint.getDegrees(),
        wristSetPoint.getDegrees() - 90);

    currentVisualizer.update(
        inputs.shoulderPosition.getDegrees() - 90,
        inputs.elbowPosition.getDegrees(),
        inputs.wristPosition.getDegrees() - 90);

    Logger.recordOutput("Arm/Mechanism2d/Setpoint", setpointVisualizer.arm);
    Logger.recordOutput("Arm/Mechanism2d/Current", currentVisualizer.arm);

    RobotState.getInstance().updateArmState(
        inputs.shoulderPosition,
        inputs.elbowPosition,
        inputs.wristPosition,
        armKinematics.calculateForwardKinematics(inputs.shoulderPosition, inputs.elbowPosition)[0],
        armKinematics.calculateForwardKinematics(inputs.shoulderPosition, inputs.elbowPosition)[1]);

    if(inputs.elbowSwitchState){
      // io.resetShoulderPosition(
      //   armKinematics.getArmAngles(ArmConstants.ArmStates.START.xTarget, 
      //   ArmConstants.ArmStates.START.yTarget, 
      //   ArmConstants.activeEncoders)[0]);

      // io.resetElbowPosition(
      //   armKinematics.getArmAngles(ArmConstants.ArmStates.START.xTarget, 
      //   ArmConstants.ArmStates.START.yTarget, 
      //   ArmConstants.activeEncoders)[1]);

      // io.resetShoulderPosition(Rotation2d.fromDegrees(-63));
      // io.resetElbowPosition(Rotation2d.fromDegrees(97));
    }

    SmartDashboard.putNumber("ShoulderSetpoint", shoulderSetPoint.getDegrees());

    SmartDashboard.putNumber("ElbowSetpoint", elbowSetPoint.getDegrees());

    SmartDashboard.putNumber("WristSetpoint", wristSetPoint.getDegrees());

    SmartDashboard.putNumber("ShoulderDeg", inputs.shoulderPosition.getDegrees());

    SmartDashboard.putNumber("ElbowDeg", inputs.elbowPosition.getDegrees());

    SmartDashboard.putNumber("WristDeg", inputs.wristPosition.getDegrees());

    SmartDashboard.putBoolean("LimitSwitch", inputs.elbowSwitchState);

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
    setpoint = pose;
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

    if(armKinematics.isValidBumpRequest(bumpSetpoint.xTarget, bumpSetpoint.yTarget)){
      setpoint = bumpSetpoint;
    } else {
      //TODO: Print out problem?
    }
  }

  public void bumpYArm(double inches){
    ArmPose bumpSetpoint = new ArmPose(setpoint.xTarget, 
                            setpoint.yTarget + Units.inchesToMeters(inches), 
                            setpoint.wristAngle);

    if(armKinematics.isValidBumpRequest(bumpSetpoint.xTarget, bumpSetpoint.yTarget)){
      setpoint = bumpSetpoint;
    } else {
      //TODO: Print out problem?
    }
  }
}
