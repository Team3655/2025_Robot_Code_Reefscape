package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

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

  public record ArmState(Rotation2d shoulderAngle, Rotation2d elbowAngle, Rotation2d wristAngle){}

  private SwerveDriveKinematics kinematics;

  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator poseEstimator;

  private SwerveModulePosition[] lastModulePositions;

  public ArmState armState;

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

    kinematics = DriveConstants.kinematics;

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
    
    armState = new ArmState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));
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

  public synchronized void updateArmState(Rotation2d shoulderAngle, Rotation2d elbowAngle, Rotation2d wristAngle){
    armState = new ArmState(shoulderAngle, elbowAngle, wristAngle);
  }

  public synchronized void resetPose(Pose2d pose) {
    odometry.resetPosition(
        getRotation(),
        lastModulePositions,
        pose);

    poseEstimator.resetPosition(
        getRotation(),
        lastModulePositions,
        pose);
  }

  public synchronized void zeroHeading() {
    odometry.resetRotation(new Rotation2d());
    poseEstimator.resetRotation(new Rotation2d());
  }

  @AutoLogOutput(key = "RobotState/Pose")
  public Pose2d getPose() {
    return new Pose2d(
      getEstimatedPose().getTranslation(),
      getRotation()
    );
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public Rotation2d getRotation() {
    return getOdometryPose().getRotation();
  }

  public Rotation2d[] getArmState(){
    Rotation2d[] state = {armState.shoulderAngle, armState.elbowAngle, armState.wristAngle};
    return state;
  }

}
