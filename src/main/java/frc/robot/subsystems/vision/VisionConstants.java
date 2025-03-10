// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Static values for the Vision subsystem */
public class VisionConstants {

  // the maximum distance a measurement will be accepted in meters
  public static final double SINGLE_TAG_MAXIMUM = 3.5;
  public static final double MULTI_TAG_MAXIMUM = 7.5;

  public static final double MAX_AMBIGUITY = 0.25;

  public static final double LINEAR_STD_DEV_FACTOR = 0.8;
  public static final double ANGULAR_STD_DEV_FACTOR = 2;

  public static final double MEGATAG2_LINEAR_FACTOR = 0.25;
  public static final double MEGATAG2_ANGULAR_FACTOR = 2;

  public static final Translation3d LEFT_ROBOT_TO_CAMERA_TRANSLATION = new Translation3d(-0.117, 0.2814, 0.2);
  public static final Rotation3d LEFT_ROBOT_TO_CAMERA_ROTATION = new Rotation3d(180,28, -15);
  public static final Transform3d LEFT_ROBOT_TO_CAMERA = new Transform3d(LEFT_ROBOT_TO_CAMERA_TRANSLATION, LEFT_ROBOT_TO_CAMERA_ROTATION);

  public static final Translation3d RIGHT_ROBOT_TO_CAMERA_TRANSLATION = new Translation3d(0.177, 0.214, 0.2);
  public static final Rotation3d RIGHT_ROBOT_TO_CAMERA_ROTATION = new Rotation3d(180,28, 15);
  public static final Transform3d RIGHT_ROBOT_TO_CAMERA = new Transform3d(RIGHT_ROBOT_TO_CAMERA_TRANSLATION, RIGHT_ROBOT_TO_CAMERA_ROTATION);

  /**
   * A record to store position data for an observation.
   * 
   * @param tx The angle from the target on the x axis.
   * @param ty The angle from the target on the y axis.
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * A Record to store data about the robots pose calculated from vision
   * 
   * @param timestamp          The time in seconds when this observation was
   *                           recorded
   * @param pose               A pose 3d representing the robot's position on the
   *                           field
   * @param ambiguity          A value between 0 and 1 representing how confident
   *                           the vision system is in the pose where 0 is very
   *                           confident and 1 is not confident
   * @param tagCount           The number of tags that were used to calculate this
   *                           pose
   * @param averageTagDistance The average distance between tags used to calculate
   *                           this pose
   * 
   * @param type               The way the pose was calculated, used to calculate
   *                           standard deviations
   */
  public static record PoseObservation(
      double timestamp,
      Pose2d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      ObservationType type) {}

  public enum ObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTON
  }
}
