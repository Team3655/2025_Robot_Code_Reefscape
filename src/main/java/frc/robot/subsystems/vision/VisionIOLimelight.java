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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

/** Uses a Limelight camera to do vision calculations. */
public class VisionIOLimelight implements VisionIO {

  private final String name;

  private NetworkTable limelightTable;
  private NetworkTableEntry tvEntry;

  public VisionIOLimelight(String name) {

    this.name = name;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    limelightTable = inst.getTable("limelight"); // Access the "limelight" table
    tvEntry = limelightTable.getEntry("tv"); // Get the "tv" entry
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    inputs.connected = isLimelightConnected();

    // Get raw AprilTag/Fiducial data
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);

    var estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

    double lowestAmbiguity = 1;
    double closestTargetDistance = 100;
    double totalTagDistance = 0.0;

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Check if there is more than one tag found
    if (fiducials.length > 1) {
      for (RawFiducial fiducial : fiducials) {
        int id = fiducial.id; // Tag ID
        double txnc = fiducial.txnc; // X offset (no crosshair)
        double tync = fiducial.tync; // Y offset (no crosshair)
        double distToCamera = fiducial.distToCamera; // Distance to camera
        double ambiguity = fiducial.ambiguity; // Tag pose ambiguity

        totalTagDistance += distToCamera;

        tagIds.add((short) id);

        // Check for best target.
        if (ambiguity < lowestAmbiguity && distToCamera < closestTargetDistance) {
          inputs.latestObservation = new TargetObservation(Rotation2d.fromDegrees(txnc), Rotation2d.fromDegrees(tync));
          lowestAmbiguity = ambiguity;
          closestTargetDistance = distToCamera;
        }
      }

    } else if (fiducials.length == 1) {

      int id = fiducials[0].id;
      double txnc = fiducials[0].txnc;
      double tync = fiducials[0].tync;
      double distToCamera = fiducials[0].distToCamera;
      double ambiguity = fiducials[0].ambiguity;

      totalTagDistance = distToCamera;

      lowestAmbiguity = ambiguity;

      tagIds.add((short) id);

      inputs.latestObservation = new TargetObservation(Rotation2d.fromDegrees(txnc), Rotation2d.fromDegrees(tync));
    }

    poseObservations
        .add(
            new PoseObservation(
                estimatedPose.timestampSeconds,
                new Pose3d(estimatedPose.pose),
                lowestAmbiguity,
                tagIds.size(),
                totalTagDistance / tagIds.size()));

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

  // methods within IO layer should be declared private
  private boolean isLimelightConnected() {
    double tv = tvEntry.getDouble(0); // Try to read the "tv" value
    return !Double.isNaN(tv) || tv != 0; // Check if the value is valid
  }
}
