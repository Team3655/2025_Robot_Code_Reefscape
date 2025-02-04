package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPose;

public class ArmConstants {

        /*
         * Kg - output to overcome gravity (output)
         * Ks - output to overcome static friction (output)
         * Kv - output per unit of target velocity (output/rps)
         * Ka - output per unit of target acceleration (output/(rps/s))
         * Kp - output per unit of error in position (output/rotation)
         * Ki - output per unit of integrated error in position (output/(rotation*s))
         * Kd - output per unit of error in velocity (output/rps)
         */

        public static final String CANBUS_NAME = "rio";

        public static final ArmEncoders activeEncoders = ArmEncoders.RELATIVE;

        public static enum ArmEncoders {
                /** Running relative encoders within the motors */
                RELATIVE,

                /** Running absolute encoders mounted to joints */
                ABSOLUTE
        }

        public static final int SHOULDER_MOTOR_ID = 30;
        public static final int SHOULDER_MOTOR_FOLLOWER_ID = 31;
        public static final int SHOULDER_CANCODER_ID = 32;
        public static final Rotation2d SHOULDER_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double SHOULDER_LENGTH_METERS = Units.inchesToMeters(21);
        public static final double SHOULDER_MASS_KG = 1;
        public static final Rotation2d SHOULDER_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
        public static final Rotation2d SHOULDER_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
        public static final double SHOULDER_REDUCTION = 7500000 / 117600;
        public static final double KG_SHOULDER = 0.6;
        public static final double KS_SHOULDER = 0.3;
        public static final double KV_SHOULDER = 0.12;
        public static final double KA_SHOULDER = 0.0;
        public static final double KP_SHOULDER = 30.0;
        public static final double KI_SHOULDER = 0.0;
        public static final double KD_SHOULDER = 0.0;
        public static final double SHOULDER_MAX_VELOCITY_RPS = 0.6;
        public static final double SHOULDER_MAX_ACCELERATION_RPS2 = 9.0;
        public static final double SHOULDER_MAX_JERK_RPS3 = 60.0;

        public static final int ELBOW_MOTOR_ID = 33;
        public static final int ELBOW_CANCODER_ID = 34;
        public static final Rotation2d ELBOW_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double ELBOW_LENGTH_METERS = Units.inchesToMeters(23);
        public static final double ELBOW_MASS_KG = 1;
        public static final Rotation2d ELBOW_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
        public static final Rotation2d ELBOW_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
        public static final double ELBOW_REDUCTION = 7500000 / 117600;
        public static final double KG_ELBOW = 0.0; // KG_ELBOW should remain 0.0 based on the arm's design
        public static final double KS_ELBOW = 0.5;
        public static final double KV_ELBOW = 0.0;
        public static final double KA_ELBOW = 0.0;
        public static final double KP_ELBOW = 30.0;
        public static final double KI_ELBOW = 0.0;
        public static final double KD_ELBOW = 0.0;
        public static final double ELBOW_MAX_VELOCITY_RPS = 1;
        public static final double ELBOW_MAX_ACCELERATION_RPS2 = 8.0;
        public static final double ELBOW_MAX_JERK_RPS3 = 60.0;

        public static final int WRIST_MOTOR_ID = 35;
        public static final int WRIST_CANCODER_ID = 36;
        public static final Rotation2d WRIST_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(8);
        public static final Rotation2d ARM_DEFAULT_SETPOINT = Rotation2d.fromDegrees(75);
        public static final double WRIST_MASS_KG = 1;
        public static final Rotation2d WRIST_MIN_ANGLE_RADS = Rotation2d.fromDegrees(90);
        public static final Rotation2d WRIST_MAX_ANGLE_RADS = Rotation2d.fromDegrees(200);
        public static final double WRIST_REDUCTION = 7500000 / 117600;
        public static final double KG_WRIST = 0.0;
        public static final double KS_WRIST = 0.0;
        public static final double KV_WRIST = 0.0;
        public static final double KA_WRIST = 0.0;
        public static final double KP_WRIST = 0.1;
        public static final double KI_WRIST = 0.0;
        public static final double KD_WRIST = 0.0;
        public static final double WRIST_MAX_VELOCITY_RPS = 0.5;
        public static final double WRIST_MAX_ACCELERATION_RPS2 = 8.0;
        public static final double WRIST_MAX_JERK_RPS3 = 60.0;

        public static final double H_TOWER_GROUND_HEIGHT_METERS = Units.inchesToMeters(31.25);
        public static final double D_ARM_HORIZONTAL_OFFSET_METERS = Units.inchesToMeters(8.25);
        public static final double TOWER_CHASSIS_HEIGHT_METERS = Units.inchesToMeters(27.5);

        public class ArmStates {

                // xTarget
                // yTarget
                // wristPosition
                public static final ArmPose START = new ArmPose(
                                Units.inchesToMeters(12),
                                Units.inchesToMeters(34.4),
                                Rotation2d.fromDegrees(0));

                public static final ArmPose FRONT_FEEDER = new ArmPose(
                                Units.inchesToMeters(16),
                                Units.inchesToMeters(34.4),
                                Rotation2d.fromDegrees(0));

                public static final ArmPose FRONT_TROUGH_REEF = new ArmPose(
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0),
                                Rotation2d.fromDegrees(0));

                public static final ArmPose FRONT_L1_REEF = new ArmPose(
                                Units.inchesToMeters(25),
                                Units.inchesToMeters(35),
                                Rotation2d.fromDegrees(0));

                public static final ArmPose FRONT_L2_REEF = new ArmPose(
                                Units.inchesToMeters(29.5),
                                Units.inchesToMeters(50),
                                Rotation2d.fromDegrees(0));

                public static final ArmPose FRONT_L3_REEF = new ArmPose(
                                Units.inchesToMeters(5),
                                Units.inchesToMeters(74),
                                Rotation2d.fromDegrees(0));

                public static final ArmPose REAR_L4_REEF = new ArmPose(
                                Units.inchesToMeters(-2.7),
                                Units.inchesToMeters(74),
                                Rotation2d.fromDegrees(90));

                public static final ArmPose REAR_L3_REEF = new ArmPose(
                                Units.inchesToMeters(-6),
                                Units.inchesToMeters(40),
                                Rotation2d.fromDegrees(0));

                // public static final ArmPose REAR_L2_REEF = new ArmPose(
                // Units.inchesToMeters(0),
                // Units.inchesToMeters(0),
                // Rotation2d.fromDegrees(0));

                // public static final ArmPose REAR_L1_REEF = new ArmPose(
                // Units.inchesToMeters(0),
                // Units.inchesToMeters(0),
                // Rotation2d.fromDegrees(0));
        }
}
