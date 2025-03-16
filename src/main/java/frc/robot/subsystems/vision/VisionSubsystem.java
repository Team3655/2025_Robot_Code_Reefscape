// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.subsystems.vision.VisionConstants.ObservationType;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIOInputsAutoLogged[] inputs;
  private final VisionIO[] io;
  private final Alert[] disconnectedAlerts;
  private AprilTagFieldLayout tagLayout;

  public VisionSubsystem(VisionIO... io) {
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged[io.length];

    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load april tags :3 !!", null);
    }

    // Create inputs
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < disconnectedAlerts.length; i++) {
      disconnectedAlerts[i] = new Alert(inputs[i].name + " is disconnected", AlertType.kWarning);
    }
  }

  public TargetObservation getLatestTargetObservation(int cameraID) {
    return new TargetObservation(inputs[cameraID].latestObservation.tx(), inputs[cameraID].latestObservation.ty());
  }

  public boolean isInsideField(PoseObservation observation) {
    // Directions are relative to driver's perspective
    // Blue Left: y = 0.7587x + 267.2
    // Blue Right: y = -0.7587x - 49.95
    // Red Left: y = 0.7587x - 474.187
    // Red Right: y = -0.7587x + 791.337

    double slope = Units.inchesToMeters(0.7587);
    double x = observation.pose().getX();
    double y = observation.pose().getY();

    boolean isInsideBlueLeft = (y < slope*x + Units.inchesToMeters(267.2));
    boolean isInsideBlueRight = (y > -slope*x + Units.inchesToMeters(49.95));
    boolean isInsideRedLeft = (y > slope*x - Units.inchesToMeters(474.187));
    boolean isInsideRedRight = (y < -slope*x + Units.inchesToMeters(791.337));
    boolean isInsideRectangle = (x > 0 && x < tagLayout.getFieldLength() && y > 0 && y < tagLayout.getFieldWidth());

    if(isInsideBlueRight && isInsideBlueLeft && isInsideRedLeft && isInsideRedRight && isInsideRectangle) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {

    // Update inputs
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Inputs/Vision/Camera " + inputs[i].name, inputs[i]);
    }

    for (int i = 0; i < io.length; i++) {
      // Check if camera is disconnected and alert if so
      disconnectedAlerts[i].set(!inputs[i].connected);

      // Logging data
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose2d> robotPoses = new LinkedList<>();
      List<Pose2d> robotPosesAccepted = new LinkedList<>();
      List<Pose2d> robotPosesRejected = new LinkedList<>();

      // Get positions of every tag that is seen
      for (int tagId : inputs[i].tagIds) {
        var tagPose = tagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        } else {
          continue;
        }
      }

      // Go through pose observations and add them to our estimated pose
      for (var observation : inputs[i].poseObservations) {

        // Check if pose should be rejected
        boolean rejectPose = observation.tagCount() > 1 ?

        // Multiple tags in observation
            observation.tagCount() == 0 // Must have at least one tag
                || observation.ambiguity() > VisionConstants.MAX_AMBIGUITY // Must be a trustworthy pose
                || observation.averageTagDistance() > VisionConstants.MULTI_TAG_MAXIMUM // Must not be too far away
                // Must be within the field
                || !isInsideField(observation)


            // Single tag in observation
            : observation.tagCount() == 0 // Must have at least one tag
                || observation.ambiguity() > VisionConstants.MAX_AMBIGUITY // Must be a trustworthy pose
                || observation.averageTagDistance() > VisionConstants.SINGLE_TAG_MAXIMUM // Must not be too far away
                // Must be within the field
                || !isInsideField(observation);

        // Add pose to list of all poses
        robotPoses.add(observation.pose());

        // Add it to its respective list and skip calculation if pose was rejected
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          continue;
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        double linearStdDev;
        double angularStdDev;

        // Calculate standard deviations
        linearStdDev = VisionConstants.LINEAR_STD_DEV_FACTOR * Math.pow(observation.averageTagDistance(), 2)
            / observation.tagCount();
        angularStdDev = VisionConstants.ANGULAR_STD_DEV_FACTOR * Math.pow(observation.averageTagDistance(), 2)
            / observation.tagCount();

        if (observation.type() == ObservationType.MEGATAG_2) {
          linearStdDev = VisionConstants.MEGATAG2_LINEAR_FACTOR * Math.pow(observation.averageTagDistance(), 2)
              / observation.tagCount();
          angularStdDev = VisionConstants.MEGATAG2_ANGULAR_FACTOR * Math.pow(observation.averageTagDistance(), 2)
              / observation.tagCount();
        }

        // Add the measurement to the poseEstimator
        RobotState.getInstance()
            .addVisionMeasurement(
                new VisionMeasurement(
                    observation.timestamp(),
                    observation.pose(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
      }

      // Log individual camera data
      Logger.recordOutput(
          "Vision/Cameras/" + inputs[i].name + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Cameras/" + inputs[i].name + "/RobotPoses",
          robotPoses.toArray(new Pose2d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Cameras/" + inputs[i].name + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose2d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Cameras/" + inputs[i].name + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose2d[robotPosesRejected.size()]));
    }
  }
}
