package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * VisionIOSim
 * 
 * Simulates the outputs of a Vision IO based off of the robots current assumed
 * position. The class returns
 * the current position of the robot if any tags are in view.
 * 
 * @author quinnhaywood
 */
public class VisionIOSim implements VisionIO {

  /**
   * Mimics the effects of poor quality messy data, aka real world data. This is
   * good for testing your
   * vision system against the horrors of reality.
   */
  private static final boolean USE_RANDOM = true;

  private Transform3d cameraTransform;
  private Rotation2d verticalFOV;
  private Rotation2d horizontalFOV;

  private Random random = new Random();

  // load the tag layout for the current year
  private static final List<AprilTag> TAGS = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTags();

  /**
   * Simulates the outputs of a Vision IO based off of the robots current assumed
   * position. The class returns
   * the current position of the robot if any tags are in view.
   * 
   * @param pose          A transform representing the relative position of the
   *                      camera to the center of the robot.
   * @param HorizontalFOV The field of view of the camera horizontally represent
   *                      as a Rotation2d
   * @param VerticalFOV   The field of view of the camera Vertically represent as
   *                      a Rotation2d
   */
  public VisionIOSim(Transform3d pose, Rotation2d HorizontalFOV, Rotation2d VerticalFOV) {
    this.cameraTransform = pose;
    this.verticalFOV = VerticalFOV;
    this.horizontalFOV = HorizontalFOV;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d robotPose) {

    // transform the robot pose by the camera transform to get the cameras pose on
    // the field
    Pose3d cameraPose = new Pose3d(robotPose).transformBy(cameraTransform);

    LinkedList<Pose3d> targetPoses = new LinkedList<>();
    LinkedList<Double> toTagDistances = new LinkedList<>();

    // set to true if there were any valid targets in view
    boolean hasValid = false;

    for (int i = 0; i < TAGS.size(); i++) {

      Pose3d tagPose = TAGS.get(i).pose;

      // get the transform from the camera to the target
      Pose3d targetRelative = tagPose.relativeTo(cameraPose);

      // use atan2 because it respects the sign of the input
      double xyAngle = Math.atan2(targetRelative.getY(), targetRelative.getX());
      double xzAngle = Math.atan2(targetRelative.getZ(), targetRelative.getX());

      boolean isCurrentValid =
          // check to see if the target is within the cameras fov
          (Math.abs(xyAngle) < horizontalFOV.getRadians() / 2.0) &&
              (Math.abs(xzAngle) < verticalFOV.getRadians() / 2.0) &&
              // assume the camera cannot see targets further than 10 meters
              targetRelative.getTranslation().getNorm() < 10.0 &&
              // if using randomness introduce the chance that the tag is not spotted based
              // off of
              // the degree of the targets angle relative to the fov
              (USE_RANDOM ? 1 >= Math.abs(random.nextGaussian() * ((xyAngle / horizontalFOV.getRadians() * 2))) : true);

      // if the current tag is valid add data to lists
      if (isCurrentValid) {
        toTagDistances.add(targetRelative.getTranslation().getNorm());
        targetPoses.add(tagPose);
      } else {
        toTagDistances.add(0.0);
      }

      // set has valid to be true if any tags are valid
      hasValid = hasValid || isCurrentValid;
    }

    // update inputs
    if (hasValid) {
      inputs.hasValidTarget = false;
      if (USE_RANDOM) {
        robotPose = robotPose.plus(
            new Transform2d(
                new Translation2d(
                    random.nextGaussian() * (0.5 / (targetPoses.size() * 3)),
                    Rotation2d.fromRadians(Math.random() * (2 * Math.PI))),
                Rotation2d.fromDegrees(random.nextGaussian() * (8 / (targetPoses.size() * 3)))));
      }
      inputs.robotPose = new Pose2d[] { robotPose };
      inputs.isNew = true;
    } else {
      inputs.hasValidTarget = false;
      inputs.robotPose = new Pose2d[] {};
      inputs.isNew = false;
    }

    inputs.targetPoses = targetPoses.toArray(Pose3d[]::new);
    inputs.toTagDistances = toTagDistances.stream().mapToDouble(d -> d).toArray();
    inputs.timestamp = MathSharedStore.getTimestamp();
  }

}