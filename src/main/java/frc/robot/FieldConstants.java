package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

  public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldLength = layout.getFieldLength();
  public static final double fieldWidth = layout.getFieldWidth();

  public static class Reef {

    public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);

    public static final Pose2d[] rightBranchPoses = new Pose2d[6];
    public static final Pose2d[] leftBranchPoses = new Pose2d[6];

    static {
      for (int i = 0; i < 6; i++) {
        Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * i)));
        double adjustX = Units.inchesToMeters(30.738);
        double adjustY = Units.inchesToMeters(6.469);

        Transform2d leftTransform = new Transform2d(adjustX, -adjustY, new Rotation2d());
        Transform2d rightTransform = new Transform2d(adjustX, adjustY, new Rotation2d());

        var leftBranch = poseDirection.transformBy(leftTransform);
        var rightBranch = poseDirection.transformBy(rightTransform);
        
        leftBranchPoses[i] = leftBranch;
        rightBranchPoses[i] = rightBranch;
      }
    }
  }

}
