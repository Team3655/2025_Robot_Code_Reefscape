// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPoseCommand extends Command {

  private Supplier<Pose2d> target;
  private DriveSubsystem drive;

  // TODO: tune
  private final ProfiledPIDController xyController = new ProfiledPIDController(
      0,
      0,
      0,
      new Constraints(1.0, 5.0));

  // TODO: tune
  private final ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0, null);

  /** Creates a new DriveToPoseCommand. */
  public DriveToPoseCommand(Supplier<Pose2d> target, DriveSubsystem drive) {
    this.target = target;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: use the length of the delta vector to reset the controller
    xyController.reset(0.0);
    // TODO: use the rotatoin of the robot to reset the controller
    thetaController.reset(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO: calculate linear velocity
    Translation2d linearVelocity = new Translation2d();

    // TODO: calculate omega
    double omega = 0.0;

    ChassisSpeeds speeds = new ChassisSpeeds(
        linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
        linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
        0.0 * DriveConstants.MAX_ANGULAR_SPEED);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            RobotState.getInstance().getEstimatedPose().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: return true if both the translation error and rotation error are small
    // enough

    return false;
  }

  private Pose2d getXYDelta() {
    // TODO: return the pose representing the difference between the current pose
    // and target pose
    // target pose - current pose
    return null;
  }
}
