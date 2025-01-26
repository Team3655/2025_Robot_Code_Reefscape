package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(19.5);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = (MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS);

  public static final Rotation2d BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.251);// (0.427);
  public static final Rotation2d BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.442); // (0.000977);
  public static final Rotation2d FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.026);// (0.115479);
  public static final Rotation2d FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(0.088); // (-0.095947);

  public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  public static final double KP_TURN = 100;
  public static final double KP_DRIVE = 0.3;
  public static final double KV_DRIVE = 0.13;

  public static final int DRIVE_CURRENT_LIMIT = 35;
  public static final int TURN_CURRENT_LIMIT = 15;

  // Used to calculate feed forward for turn speed in 2nd order dynamics calc.
  public static final double TURN_kA = 0;
  public static final double TURN_kS = 0.05;
  public static final double TURN_kV = 0.1;

  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
  };
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

}
