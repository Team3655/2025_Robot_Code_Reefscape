// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO[] limelights;
  private final VisionIOInputsAutoLogged[] llInputs;

  private ArrayList<Pose2d> acceptedPoses = new ArrayList<>();
  private ArrayList<Pose2d> rejectedPoses = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO... limelights) {
    this.limelights = limelights;
    // create and fill a list of autologged inputs;
    llInputs = new VisionIOInputsAutoLogged[limelights.length];
    for (int i = 0; i < limelights.length; i++) {
      llInputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    rejectedPoses.clear();
    acceptedPoses.clear();

    for (int i = 0; i < limelights.length; i++) {

      limelights[i].updateInputs(llInputs[i], RobotState.getInstance().getEstimatedPose());
      Logger.processInputs("Vision/" + limelights[i].getName(), llInputs[i]);

      for (int j = 0; j < llInputs[i].robotPose.length; j++) {

        // basic checks to avoid errors
        if (!llInputs[i].isNew
            || !llInputs[i].hasValidTarget
            || llInputs[i].robotPose[j] == null
            || llInputs[i].targetPoses.length < 1) {
          continue;
        }

        // calculate the average distance to the target
        double avgDist = 0.0;
        for (double d : llInputs[i].toTagDistances) {
          avgDist += d;
        }
        avgDist /= llInputs[i].targetPoses.length;

        // if the camera to target distance is too great reject the measurement,
        // depending on the number of tags in view use a different maximum
        if (avgDist >= (llInputs[i].targetPoses.length > 1
            ? VisionConstants.MULTI_TAG_MAXIMUM
            : VisionConstants.SINGLE_TAG_MAXIMUM)
            // reject measure if outside of field
            || !isInField(llInputs[i].robotPose[j])) {
          // add pose to rejected poses for logging
          rejectedPoses.add(llInputs[i].robotPose[j]);
          continue;
        }

        // calculate the standard deviations for the robots x and y position
        double xyStdDev = Math.pow(avgDist, 1.6
            * VisionConstants.TRANSLATION_COEFFICIENT
            / (double) llInputs[i].targetPoses.length);

        // calculate the standard deviations for the robots rotation
        double thetaStdDev = Math.pow(avgDist, 1.6)
            * VisionConstants.ROTATION_COEFFICIENT
            / (double) llInputs[i].targetPoses.length;

        // add pose to the accepted poses for logging
        acceptedPoses.add(llInputs[i].robotPose[j]);

        // offer a new measurement to the RobotState
        RobotState.getInstance().addVisionMeasurement(
            new VisionMeasurement(
                llInputs[i].timestamp,
                llInputs[i].robotPose[j],
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
      }
    }

    Logger.recordOutput("Vision/acceptedPoses", acceptedPoses.toArray(Pose2d[]::new));
    Logger.recordOutput("Vision/rejectedPoses", rejectedPoses.toArray(Pose2d[]::new));
  }

  /**
   * Checks if the provided pose is within the playing field
   * 
   * @param pose the pose to check
   * @return true: if the pose is within the field
   */
  private static boolean isInField(Pose2d pose) {
    if (pose.getX() >= 0
        && pose.getX() <= Units.feetToMeters(54 + (3.0 / 12.0))
        && pose.getY() >= 0
        && pose.getY() <= Units.feetToMeters(26 + (3.0 / 12.0))) {
      return true;
    }
    return false;
  }

}
