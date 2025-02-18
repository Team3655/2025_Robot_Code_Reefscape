// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Static values for the Vision subsystem */
public class VisionConstants {

  // the maximum distance a measurement will be accepted in meters
  public static final double SINGLE_TAG_MAXIMUM = 4.5;
  public static final double MULTI_TAG_MAXIMUM = 7.5;

  public static final double MAX_AMBIGUITY = 0.8;

  public static final double LINEAR_STD_DEV_FACTOR = 0.4;
  public static final double ANGULAR_STD_DEV_FACTOR = 0.3;

  public static final Translation3d LEFT_ROBOT_TO_CAMERA_TRANSLATION = new Translation3d(0, 0, 0);
  public static final Rotation3d LEFT_ROBOT_TO_CAMERA_ROTATION = new Rotation3d(0, 0, 0);
  public static final Transform3d LEFT_ROBOT_TO_CAMERA = new Transform3d(LEFT_ROBOT_TO_CAMERA_TRANSLATION,
      LEFT_ROBOT_TO_CAMERA_ROTATION);

  /**
   * A record to store position data for an observation.
   * 
   * @param tx The angle from the target on the x axis.
   * @param ty The angle from the target on the y axis.
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
  }

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
   */
  public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount,
      double averageTagDistance) {
  }
}
