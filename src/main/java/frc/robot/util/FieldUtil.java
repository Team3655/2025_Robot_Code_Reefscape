package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveConstants;

public class FieldUtil {

  public AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public HashMap<String, Pose2d> reefPoses = new HashMap<String, Pose2d>();
  private int[] reefTags = new int[6];

  private record PoseOffset(double horizontalOffset, double forwardOffset) {
  }

  @SuppressWarnings("unchecked")
  private Pair<Transform2d, Transform2d>[] offsets = new Pair[6];

  public FieldUtil() {

    PoseOffset left1;
    PoseOffset left2;
    PoseOffset left3;
    PoseOffset left4;
    PoseOffset left5;
    PoseOffset left6;

    PoseOffset right1;
    PoseOffset right2;
    PoseOffset right3;
    PoseOffset right4;
    PoseOffset right5;
    PoseOffset right6;

    if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
      reefTags[0] = 18;
      reefTags[1] = 19;
      reefTags[2] = 20;
      reefTags[3] = 21;
      reefTags[4] = 22;
      reefTags[5] = 17;
      /*
       * INCREASING THE HORIZONTAL OFFSET MOVES YOUR ROBOT RIGHT (ROBOT RELATIVE)
       * 
       *  INCREASING THE FORWARD OFFSET MOVES YOUR ROBOT FORWARD
       * 
       * BLUE SIDE OFFSETS
       */
      left1 = new PoseOffset(Units.inchesToMeters(-1.5), 0);
      left2 = new PoseOffset(Units.inchesToMeters(0), 0);
      left3 = new PoseOffset(Units.inchesToMeters(0), 0);
      left4 = new PoseOffset(Units.inchesToMeters(-1), 0);
      left5 = new PoseOffset(Units.inchesToMeters(0), 0);
      left6 = new PoseOffset(Units.inchesToMeters(0), 0);

      right1 = new PoseOffset(Units.inchesToMeters(-0.5), 0);
      right2 = new PoseOffset(Units.inchesToMeters(0), 0);
      right3 = new PoseOffset(Units.inchesToMeters(-1), 0);
      right4 = new PoseOffset(Units.inchesToMeters(-0.5), 0);
      right5 = new PoseOffset(Units.inchesToMeters(0), 0);
      right6 = new PoseOffset(Units.inchesToMeters(0), 0);
    } else {
      reefTags[0] = 7;
      reefTags[1] = 6;
      reefTags[2] = 11;
      reefTags[3] = 10;
      reefTags[4] = 9;
      reefTags[5] = 8;

      // RED SIDE OFFSETS
      left1 = new PoseOffset(Units.inchesToMeters(0), 0);
      left2 = new PoseOffset(Units.inchesToMeters(0), 0);
      left3 = new PoseOffset(Units.inchesToMeters(0), 0);
      left4 = new PoseOffset(Units.inchesToMeters(1), 0);
      left5 = new PoseOffset(Units.inchesToMeters(0), 0);
      left6 = new PoseOffset(Units.inchesToMeters(0), 0);

      right1 = new PoseOffset(Units.inchesToMeters(0), 0);
      right2 = new PoseOffset(Units.inchesToMeters(0), 0);
      right3 = new PoseOffset(Units.inchesToMeters(1), 0);
      right4 = new PoseOffset(Units.inchesToMeters(0), 0);
      right5 = new PoseOffset(Units.inchesToMeters(1), 0);
      right6 = new PoseOffset(Units.inchesToMeters(-1.5), 0);
    }

    offsets[0] = new Pair<Transform2d, Transform2d>(
        new Transform2d(left1.forwardOffset * -1, left1.horizontalOffset, Rotation2d.kZero),
        new Transform2d(right1.forwardOffset * -1, right1.horizontalOffset, Rotation2d.kZero));
    offsets[1] = new Pair<Transform2d, Transform2d>(
        new Transform2d(left2.forwardOffset * -1, left2.horizontalOffset, Rotation2d.kZero),
        new Transform2d(right2.forwardOffset * -1, right2.horizontalOffset, Rotation2d.kZero));
    offsets[2] = new Pair<Transform2d, Transform2d>(
        new Transform2d(left3.forwardOffset * -1, left3.horizontalOffset, Rotation2d.kZero),
        new Transform2d(right3.forwardOffset * -1, right3.horizontalOffset, Rotation2d.kZero));
    offsets[3] = new Pair<Transform2d, Transform2d>(
        new Transform2d(left4.forwardOffset * -1, left4.horizontalOffset, Rotation2d.kZero),
        new Transform2d(right4.forwardOffset * -1, right4.horizontalOffset, Rotation2d.kZero));
    offsets[4] = new Pair<Transform2d, Transform2d>(
        new Transform2d(left5.forwardOffset * -1, left5.horizontalOffset, Rotation2d.kZero),
        new Transform2d(right5.forwardOffset * -1, right5.horizontalOffset, Rotation2d.kZero));
    offsets[5] = new Pair<Transform2d, Transform2d>(
        new Transform2d(left6.forwardOffset * -1, left6.horizontalOffset, Rotation2d.kZero),
        new Transform2d(right6.forwardOffset * -1, right6.horizontalOffset, Rotation2d.kZero));

    for (int i = 0; i <= 5; i++) {
      Pose2d tagPose = tagLayout.getTagPose(reefTags[i]).get().toPose2d();
      Pose2d leftAlignment;
      Pose2d rightAlignment;

      boolean shouldFlip = i == 2 || i == 3 || i == 4;

      // Intake is not centered on the robot
      double intakeOffset = Units.inchesToMeters(0);

      if (!shouldFlip) {
        leftAlignment = tagPose.transformBy(new Transform2d(DriveConstants.BUMPER_WIDTH_X / 2,
            -Units.inchesToMeters(6.47) - intakeOffset, Rotation2d.kZero));
        leftAlignment = leftAlignment.transformBy(offsets[i].getFirst());

        rightAlignment = tagPose.transformBy(new Transform2d(DriveConstants.BUMPER_WIDTH_X / 2,
            Units.inchesToMeters(6.47) - intakeOffset, Rotation2d.kZero));
        rightAlignment = rightAlignment.transformBy(offsets[i].getSecond());
      } else {

        leftAlignment = tagPose.transformBy(new Transform2d(DriveConstants.BUMPER_WIDTH_X / 2,
            Units.inchesToMeters(6.47) - intakeOffset, Rotation2d.kZero));
        leftAlignment = leftAlignment.transformBy(offsets[i].getFirst());

        rightAlignment = tagPose.transformBy(new Transform2d(DriveConstants.BUMPER_WIDTH_X / 2,
            -Units.inchesToMeters(6.47) - intakeOffset, Rotation2d.kZero));
        rightAlignment = rightAlignment.transformBy(offsets[i].getSecond());
      }

      reefPoses.put("Left" + (i + 1), leftAlignment);
      reefPoses.put("Right" + (i + 1), rightAlignment);
    }
  }

}
