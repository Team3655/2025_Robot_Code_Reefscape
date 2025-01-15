package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public static final double MAX_LINEAR_SPEED = (0.4) * Units.feetToMeters(19.5);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = (0.2) * (MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS);
  public static final String CANIVORE_NAME = "ctre";

  public static final Rotation2d BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.254883);//(0.427);
  public static final Rotation2d BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.066650); //(0.000977);
  public static final Rotation2d FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.471680);//(0.115479);
  public static final Rotation2d FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(.414062); //(-0.095947);

  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
  };
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

}
