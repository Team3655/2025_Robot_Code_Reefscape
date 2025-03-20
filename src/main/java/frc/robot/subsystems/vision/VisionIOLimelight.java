// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants.ObservationType;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;
import frc.robot.util.LimelightHelpers;

/** Uses a Limelight camera to do vision calculations. */
public class VisionIOLimelight implements VisionIO {

  public final String name;
  private final NetworkTable table;
  private final DoubleSubscriber latencySubscriber;

  public VisionIOLimelight(String name) {
    this.name = name;
    table = NetworkTableInstance.getDefault().getTable(name);

    // tl represents the limelights latency contribution
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    
    LimelightHelpers.SetIMUMode(name, 0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    inputs.name = name;

    inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    LimelightHelpers.SetRobotOrientation(name, RobotState.getInstance().getEstimatedPose().getRotation().getDegrees(),0, 0, 0, 0, 0);
    NetworkTableInstance.getDefault().flush();

    var rawFiducials = LimelightHelpers.getRawFiducials(name);
    var estimatedPoseMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    var estimatedPoseMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    Set<Integer> tagIds = new HashSet<>();
    List<Double> tagAmbiguities = new ArrayList<>();
    double totalTagDistance = 0.0;
    double totalPoseAmbiguity = 0.0;

    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var tag : rawFiducials) {
      tagIds.add(tag.id);
      tagAmbiguities.add(tag.ambiguity);

      totalTagDistance += tag.distToRobot;
      totalPoseAmbiguity += tag.ambiguity;
    }

    if (estimatedPoseMT1 != null) {
      poseObservations.add(new PoseObservation(
          estimatedPoseMT1.timestampSeconds,
          estimatedPoseMT1.pose,
          totalPoseAmbiguity / tagAmbiguities.size(),
          tagIds.size(),
          estimatedPoseMT1.avgTagDist,
          ObservationType.MEGATAG_1));
    }

    if (estimatedPoseMT2 != null && DriverStation.isEnabled()) {
      
      poseObservations.add(new PoseObservation(
          estimatedPoseMT2.timestampSeconds,
          estimatedPoseMT2.pose,
          0.0,
          tagIds.size(),
          totalTagDistance / tagIds.size(),
          ObservationType.MEGATAG_2));
    }

    inputs.latestObservation = new TargetObservation(
        Rotation2d.fromDegrees(LimelightHelpers.getTX(name)),
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
