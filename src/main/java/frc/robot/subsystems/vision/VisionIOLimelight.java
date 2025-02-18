// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

/** Uses a Limelight camera to do vision calculations. */
public class VisionIOLimelight implements VisionIO {

  private final String name;
  double lastTx;

  public VisionIOLimelight(String name) {

    this.name = name;

  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    inputs.connected = true;

    LimelightHelpers.SetIMUMode(name, 0);
    // Get raw AprilTag/Fiducial data
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
    LimelightHelpers.PoseEstimate estimatedPose;

    estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

    double totalAmbiguity = 0;
    double totalTagDistance = 0.0;

    Set<Short> tagIds = new HashSet<>();
    Set<Short> ambiguities = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToCamera = fiducial.distToCamera; // Distance to camera
      double ambiguity = fiducial.ambiguity; // Tag pose ambiguity

      totalTagDistance += distToCamera;

      tagIds.add((short) id);

      totalAmbiguity += ambiguity;
      ambiguities.add((short) ambiguity);
    }

    if (estimatedPose != null) {
      poseObservations
          .add(
              new PoseObservation(
                  estimatedPose.timestampSeconds,
                  new Pose3d(estimatedPose.pose),
                  totalAmbiguity / ambiguities.size(),
                  tagIds.size(),
                  totalTagDistance / tagIds.size()));
    } else {
      poseObservations.add(new PoseObservation(0, new Pose3d(), 0, 0, 0));
    }

    inputs.latestObservation = new TargetObservation(Rotation2d.fromDegrees(LimelightHelpers.getTX(name)),
        Rotation2d.fromDegrees(LimelightHelpers.getTY(name)));

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
