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

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.JoystickUtils;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    Logger.recordOutput("Drive/Commands/joystick magnitude", linearMagnitude);

    // Square magnitude for more precise control
    linearMagnitude = JoystickUtils.curveInput(linearMagnitude, DEADBAND);

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {

          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          double omega = JoystickUtils.curveInput(omegaSupplier.getAsDouble(), DEADBAND);

          Logger.recordOutput("Drive/Commands/omega", omega);
          Logger.recordOutput("Drive/Commands/linear velocity", linearVelocity);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
                  omega * DriveConstants.MAX_ANGULAR_SPEED);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
                  
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? RobotState.getInstance().getRotation().plus(new Rotation2d(Math.PI))
                      : RobotState.getInstance().getRotation()));
        },
        drive);
  }
}
