package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class RobotState {

  public record OdometryMeasurement(
      double timestamp,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions) {
  }

  public record VisionMeasurement(
      double timestamp,
      Pose2d pose,
      Matrix<N3, N1> stdDevs) {
  }

  private SwerveDriveKinematics kinematics;

  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator poseEstimator;

  private SwerveModulePosition[] lastModulePositions;

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {

    lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    kinematics = new SwerveDriveKinematics(
        new Translation2d[] {
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d()
        });

    odometry = new SwerveDriveOdometry(
        kinematics,
        new Rotation2d(),
        lastModulePositions);

    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        lastModulePositions,
        new Pose2d(),
        VecBuilder.fill(Units.inchesToMeters(2.0), Units.inchesToMeters(2.0), Units.degreesToRadians(2.0)),
        VecBuilder.fill(0.5, 0.5, 0.5));
  }

  public synchronized void addOdometryMeasurement(OdometryMeasurement measurement) {
    odometry.update(
        measurement.gyroAngle,
        measurement.wheelPositions);

    poseEstimator.updateWithTime(
        measurement.timestamp,
        measurement.gyroAngle,
        measurement.wheelPositions);
  }

  public synchronized void addVisionMeasurement(VisionMeasurement measurement) {
    poseEstimator.addVisionMeasurement(
        measurement.pose,
        measurement.timestamp,
        measurement.stdDevs);
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        pose.getRotation(),
        lastModulePositions,
        pose);

    poseEstimator.resetPosition(
        pose.getRotation(),
        lastModulePositions,
        pose);
  }

  public void zeroEstimation(){
    poseEstimator.resetPosition(Rotation2d.fromDegrees(0), lastModulePositions, getEstimatedPose());
    odometry.resetPosition(Rotation2d.fromDegrees(0), lastModulePositions, getEstimatedPose());
  } 

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

}
