package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PhysicsUtil;

/** Static values for the drive subsystem */
public class DriveConstants {


  public static final double ODOMETRY_FREQUENCY = 250.0;

  // 16.9 rot/s of wheel -> 5.409929 m/s -> 17.7 ft/s
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(17.7);

  // TODO: get real bumper width
  public static final double BUMPER_WIDTH_X = Units.inchesToMeters(32.5);
  public static final double BUMPER_WIDTH_Y = Units.inchesToMeters(32.5);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = (MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS);

  public static final Rotation2d PROTOBOT_BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.248047);
  public static final Rotation2d PROTOBOT_BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.431152);
  public static final Rotation2d PROTOBOT_FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.027344);
  public static final Rotation2d PROTOBOT_FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(0.087646);

  public static final Rotation2d COMPBOT_BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(0.446533);
  public static final Rotation2d COMPBOT_BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.184326);
  public static final Rotation2d COMPBOT_FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.268066);
  public static final Rotation2d COMPBOT_FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.105469);


  public static final double PROTOBOT_DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);

  public static final double COMPBOT_DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
  
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  public static final double BATTERY_MASS_KG = Units.lbsToKilograms(13.95);
  public static final double BUMPER_MASS_KG = Units.lbsToKilograms(13.00);

  // TODO: Update chassis mass as compbot is built
  public static final double COMPBOT_CHASSIS_MASS_KG = Units.lbsToKilograms(46.60);
  public static final double COMPBOT_MASS_KG = COMPBOT_CHASSIS_MASS_KG + BUMPER_MASS_KG + BATTERY_MASS_KG;
  public static final double COMPBOT_MOI = PhysicsUtil.estimateRobotMOI(
    COMPBOT_MASS_KG, 
    BUMPER_WIDTH_X, 
    BUMPER_WIDTH_Y);

  public static final double PROTOBOT_CHASSIS_MASS_KG = Units.lbsToKilograms(100);
  public static final double PROTOBOT_MASS_KG = PROTOBOT_CHASSIS_MASS_KG + BUMPER_MASS_KG + BATTERY_MASS_KG;
  public static final double PROTOBOT_MOI = PhysicsUtil.estimateRobotMOI(
      PROTOBOT_MASS_KG, 
      BUMPER_WIDTH_X, 
      BUMPER_WIDTH_Y);

  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.016);
  public static final double WHEEL_COF = 1.5;

  public static final double KP_TURN = 100;
  public static final double KP_DRIVE = 0.3;
  public static final double KV_DRIVE = 0.70; // 12V/max speed roughly, 12/(16.9 rot/s) = .71
  public static final double KS_DRIVE = 0.0;

  public static final int DRIVE_CURRENT_LIMIT = 155;
  public static final int TURN_CURRENT_LIMIT = 15;

  // Used to calculate feed forward for turn speed in 2nd order dynamics calc.
  public static final double TURN_kA = 0;
  public static final double TURN_kS = 0.05;
  public static final double TURN_kV = 0.1;

  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      // FL FR BL BR
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
  };

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

}