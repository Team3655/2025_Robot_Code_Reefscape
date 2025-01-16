package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final String CANBUS_NAME = "bus";

    // TODO: make these real values


    public static final Rotation2d SHOULDER_ENCODER_OFFSET = Rotation2d.fromRotations(0);
    public static final double SHOULDER_LENGTH_METERS = Units.inchesToMeters(36);
    public static final double SHOULDER_MASS_KG = 1;
    public static final Rotation2d SHOULDER_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
    public static final Rotation2d SHOULDER_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
    public static final double SHOULDER_REDUCTION = 1 / 1;

    public static final Rotation2d ELBOW_ENCODER_OFFSET = Rotation2d.fromRotations(0);
    public static final double ELBOW_LENGTH_METERS = Units.inchesToMeters(24);
    public static final double ELBOW_MASS_KG = 1;
    public static final Rotation2d ELBOW_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
    public static final Rotation2d ELBOW_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
    public static final double ELBOW_REDUCTION = 1 / 1;

    public static final Rotation2d WRIST_ENCODER_OFFSET = Rotation2d.fromRotations(0);
    public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(8);
    public static final Rotation2d ARM_DEFAULT_SETPOINT = Rotation2d.fromDegrees(75);
    public static final double WRIST_MASS_KG = 1;
    public static final Rotation2d WRIST_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
    public static final Rotation2d WRIST_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
    public static final double WRIST_REDUCTION = 1 / 1;



}
