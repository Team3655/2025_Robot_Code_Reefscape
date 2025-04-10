package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveConstants;

public class RobotState {

  public record OdometryMeasurement(
      double timestamp,
      Rotation2d gyroRotation,
      SwerveModulePosition[] moduleDeltas,
      SwerveModulePosition[] wheelPositions) {
  }

  public record VisionMeasurement(
      double timestamp,
      Pose2d pose,
      Matrix<N3, N1> stdDevs) {
  }

  public record ArmState(
      Rotation2d shoulderAngle,
      Rotation2d elbowAngle,
      Rotation2d wristAngle,
      double xPosition,
      double yPosition) {
  }

  private SwerveDriveKinematics kinematics;

  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator poseEstimator;

  public SwerveModulePosition[] lastModulePositions;
  public Rotation2d rawGyroRotation;

  private ArmState armState;

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

    rawGyroRotation = new Rotation2d();

    armState = new ArmState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0, 0);
  }

  public synchronized void addOdometryMeasurement(OdometryMeasurement measurement) {
    // Update gyro angle
    if (measurement.gyroRotation != null) {
      // Use the real gyro angle
      rawGyroRotation = measurement.gyroRotation;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = DriveConstants.kinematics.toTwist2d(measurement.moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    odometry.update(
        rawGyroRotation,
        measurement.wheelPositions);

    poseEstimator.updateWithTime(
        measurement.timestamp,
        rawGyroRotation,
        measurement.wheelPositions);
  }

  public synchronized void addVisionMeasurement(VisionMeasurement measurement) {
    poseEstimator.addVisionMeasurement(
        measurement.pose,
        measurement.timestamp,
        measurement.stdDevs);
  }

  public synchronized void updateArmState(
      Rotation2d shoulderAngle,
      Rotation2d elbowAngle,
      Rotation2d wristAngle,
      double xPosition,
      double yPosition) {

    armState = new ArmState(shoulderAngle, elbowAngle, wristAngle, xPosition, yPosition);
  }

  public synchronized void resetPose(Pose2d pose) {
    odometry.resetPosition(
        rawGyroRotation,
        lastModulePositions,
        pose);

    poseEstimator.resetPosition(
        rawGyroRotation,
        lastModulePositions,
        pose);
  }

  public synchronized void zeroHeading() {
    resetPose(new Pose2d(
        poseEstimator.getEstimatedPosition().getX(),
        poseEstimator.getEstimatedPosition().getY(),
        new Rotation2d()));
  }

  public Rotation2d getRotation() {
    return getOdometryPose().getRotation();
  }

  @AutoLogOutput(key = "RobotState/Pose")
  public Pose2d getPose() {
    return new Pose2d(
        getEstimatedPose().getTranslation(),
        getRotation());
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  @AutoLogOutput(key = "RobotState/ArmPose")
  public ArmState getArmState() {
    return armState;
  }

  @AutoLogOutput(key = "RobotState/ReefSextant")
  public int getReefSextant(){

    Translation2d blueReefPosition = new Translation2d(4, 4);
    Translation2d redReefPosition = new Translation2d(13, 4);

    Translation2d reefPosition = DriverStation.getAlliance().get().equals(Alliance.Red) ? redReefPosition : blueReefPosition; 

    double angle = reefPosition.minus(getEstimatedPose().getTranslation()).getAngle().getDegrees();

    switch (DriverStation.getAlliance().get()) {
      case Blue:
      if(angle < 30 && angle > -30){
        return 1;
      } else if(angle < 90 && angle > 30){
        return 6;
      } else if(angle < 150 && angle > 90){
        return 5;
      } else if(angle < 180 && angle > 150){
        return 4;
      } else if(angle < -150 && angle > -180){
        return 4;
      } else if(angle < -90 && angle > -150){
        return 3;
      } else if(angle < -30 && angle > -90){
        return 2;
      }
      case Red:
      if(angle < 30 && angle > -30){
        return 4;
      } else if(angle < 90 && angle > 30){
        return 3;
      } else if(angle < 150 && angle > 90){
        return 2;
      } else if(angle < 180 && angle > 150){
        return 1;
      } else if(angle < -150 && angle > -180){
        return 1;
      } else if(angle < -90 && angle > -150){
        return 6;
      } else if(angle < -30 && angle > -90){
        return 5;
      }
      default:
      DriverStation.reportError("Your angle is not even real brah. How did we get here", false);
      return 0;
    }
  }

}
