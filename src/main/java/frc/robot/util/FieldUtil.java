package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class FieldUtil {

  public AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public HashMap<String, Pose2d> reefPoses = new HashMap<String, Pose2d>();

  private Pose2d reefPose(double xPosition, double yPosition, double angle) {
    return new Pose2d(new Translation2d(xPosition, yPosition), Rotation2d.fromDegrees(angle));
  }

  public FieldUtil() {
    if (Constants.alliance.equals(Alliance.Blue)) {
      reefPoses.put("Left1", reefPose(3.195, 4.221, 180));
      reefPoses.put("Right1", reefPose(3.195, 3.892, 180));
      reefPoses.put("Left2", reefPose(4.003, 5.249, 120));
      reefPoses.put("Right2", reefPose(3.738, 5.069, 120));
      reefPoses.put("Left3", reefPose(5.008, 5.187, 60));
      reefPoses.put("Right3", reefPose(5.307, 5.047, 60));
      reefPoses.put("Left4", reefPose(5.760, 4.145, 0));
      reefPoses.put("Right4", reefPose(5.760, 3.843, 0));
      reefPoses.put("Left5", reefPose(4.946, 2.822, -60));
      reefPoses.put("Right5", reefPose(5.232, 2.995, -60));
      reefPoses.put("Left6", reefPose(3.662, 3.011, -120));
      reefPoses.put("Right6", reefPose(3.969, 2.850, -120));
    } else {
      reefPoses.put("Left1", reefPose(14.330, 3.855, 0));
      reefPoses.put("Right1", reefPose(14.330, 4.156, 0));
      reefPoses.put("Left2", reefPose(13.537, 2.834, -60));
      reefPoses.put("Right2", reefPose(13.773, 2.986, -60));
      reefPoses.put("Left3", reefPose(12.541, 2.880, -120));
      reefPoses.put("Right3", reefPose(12.292, 3.036, -120));
      reefPoses.put("Left4", reefPose(11.800, 3.891, 180));
      reefPoses.put("Right4", reefPose(11.800, 4.212, 180));
      reefPoses.put("Left5", reefPose(12.297, 5.071, 120));
      reefPoses.put("Right5", reefPose(12.589, 5.233, 120));
      reefPoses.put("Left6", reefPose(13.863, 5.013, 60));
      reefPoses.put("Right6", reefPose(13.583, 5.166, 60));
    }
  }

}
