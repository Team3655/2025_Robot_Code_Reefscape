package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPose;

public class ArmConstants {
  public static final String CANBUS_NAME = "rio";

  public static final ArmEncoders activeEncoders = ArmEncoders.ABSOLUTE;

  public static enum ArmEncoders {
    /** Running relative encoders within the motors */
    RELATIVE,

    /** Running absolute encoders mounted to joints */
    ABSOLUTE
  }

  // TODO: make these real values

  public static final int SHOULDER_CANCODER_ID = 0;
  public static final int SHOULDER_MOTOR_ID = 0;
  public static final Rotation2d SHOULDER_ENCODER_OFFSET = Rotation2d.fromRotations(0);
  public static final double SHOULDER_LENGTH_METERS = Units.inchesToMeters(22);
  public static final double SHOULDER_MASS_KG = 1;
  public static final Rotation2d SHOULDER_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
  public static final Rotation2d SHOULDER_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
  public static final double SHOULDER_REDUCTION = 1 / 1;
  public static final double KP_SHOULDER = 0.1;
  public static final double KD_SHOULDER = 0.0;

  public static final int ELBOW_CANCODER_ID = 0;
  public static final int ELBOW_MOTOR_ID = 0;
  public static final Rotation2d ELBOW_ENCODER_OFFSET = Rotation2d.fromRotations(0);
  public static final double ELBOW_LENGTH_METERS = Units.inchesToMeters(24);
  public static final double ELBOW_MASS_KG = 1;
  public static final Rotation2d ELBOW_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
  public static final Rotation2d ELBOW_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
  public static final double ELBOW_REDUCTION = 1 / 1;
  public static final double KP_ELBOW = 0.1;
  public static final double KD_ELBOW = 0.0;

  public static final int WRIST_CANCODER_ID = 0;
  public static final int WRIST_MOTOR_ID = 0;
  public static final Rotation2d WRIST_ENCODER_OFFSET = Rotation2d.fromRotations(0);
  public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(8);
  public static final Rotation2d ARM_DEFAULT_SETPOINT = Rotation2d.fromDegrees(75);
  public static final double WRIST_MASS_KG = 1;
  public static final Rotation2d WRIST_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
  public static final Rotation2d WRIST_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
  public static final double WRIST_REDUCTION = 1 / 1;
  public static final double KP_WRIST = 0.1;
  public static final double KD_WRIST = 0.0;

  public static final double H_TOWER_GROUND_HEIGHT_METERS = Units.inchesToMeters(32.0);
  public static final double D_ARM_HORIZTONAL_OFFSET_METERS = Units.inchesToMeters(6.0);
  public static final double TOWER_CHASSIS_HEIGHT_METERS = Units.inchesToMeters(24.0);

  public class ArmStates {
    
    // xTarget
    // yTarget
    // wristPosition
    public static final ArmPose START = new ArmPose(
        Units.inchesToMeters(11),
        Units.inchesToMeters(35),
        Rotation2d.fromDegrees(0));

    public static final ArmPose FRONT_FEEDER = new ArmPose(
        Units.inchesToMeters(31),
        Units.inchesToMeters(36),
        Rotation2d.fromDegrees(0));

    public static final ArmPose REAR_L4_REEF = new ArmPose(
        Units.inchesToMeters(0),
        Units.inchesToMeters(0),
        Rotation2d.fromDegrees(0));

    public static final ArmPose REAR_L3_REEF = new ArmPose(
        Units.inchesToMeters(0),
        Units.inchesToMeters(0),
        Rotation2d.fromDegrees(0));

    public static final ArmPose REAR_L2_REEF = new ArmPose(
        Units.inchesToMeters(0),
        Units.inchesToMeters(0),
        Rotation2d.fromDegrees(0));

    public static final ArmPose REAR_L1_REEF = new ArmPose(
        Units.inchesToMeters(0),
        Units.inchesToMeters(0),
        Rotation2d.fromDegrees(0));
  }
}
