// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;

/** The interface for logging vision data */
public interface VisionIO {

  @AutoLog
  public class VisionIOInputs {
    public String name;
    public boolean connected = false;
    public TargetObservation latestObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {
  }

}
