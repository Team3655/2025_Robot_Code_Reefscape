package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
        public static final double SHOULDER_LENGTH_METERS = Units.inchesToMeters(22.75);
        public static final double SHOULDER_MASS_KG = 1;
        public static final Rotation2d SHOULDER_MIN_ANGLE_RADS = Rotation2d.fromDegrees(-67);
        public static final Rotation2d SHOULDER_MAX_ANGLE_RADS = Rotation2d.fromDegrees(90);
        public static final double SHOULDER_REDUCTION = 63; //63.7755
        public static final double KG_SHOULDER = 0.70;
        public static final double KS_SHOULDER = 3.0;
        public static final double KV_SHOULDER = 0.0;
        public static final double KA_SHOULDER = 0.0;
        public static final double KP_SHOULDER = 100.0;
        public static final double KI_SHOULDER = 0.0;
        public static final double KD_SHOULDER = 0.0;
        public static final double SHOULDER_MAX_VELOCITY_RPS = 0.3;
        public static final double SHOULDER_MAX_ACCELERATION_RPS2 = 0.4;
        public static final double SHOULDER_MAX_JERK_RPS3 = 60.0;

        public static final int ELBOW_MOTOR_ID = 33;
        public static final int ELBOW_MOTOR_FOLLOWER_ID = 34;
        public static final int ELBOW_CANCODER_ID = 35;
        public static final Rotation2d ELBOW_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double ELBOW_LENGTH_METERS = Units.inchesToMeters(24.75);
        public static final double ELBOW_MASS_KG = 1;
        public static final Rotation2d ELBOW_MIN_ANGLE_RADS = Rotation2d.fromDegrees(0);
        public static final Rotation2d ELBOW_MAX_ANGLE_RADS = Rotation2d.fromDegrees(180);
        public static final double ELBOW_REDUCTION = 63.7755;
        public static final double KG_ELBOW = 0.8; 
        public static final double KS_ELBOW = 1.8;
        public static final double KV_ELBOW = 0.0;
        public static final double KA_ELBOW = 0.0;
        public static final double KP_ELBOW = 100;
        public static final double KI_ELBOW = 0.0;
        public static final double KD_ELBOW = 0.0;
        public static final double ELBOW_MAX_VELOCITY_RPS = 0.4;
        public static final double ELBOW_MAX_ACCELERATION_RPS2 = 0.4;
        public static final double ELBOW_MAX_JERK_RPS3 = 60.0;

        public static final int WRIST_MOTOR_ID = 36;
        public static final int WRIST_CANCODER_ID = 37;
        public static final Rotation2d WRIST_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(8);
        public static final Rotation2d ARM_DEFAULT_SETPOINT = Rotation2d.fromDegrees(75);
        public static final double WRIST_MASS_KG = 1;
        public static final Rotation2d WRIST_MIN_ANGLE_RADS = Rotation2d.fromDegrees(-360);
        public static final Rotation2d WRIST_MAX_ANGLE_RADS = Rotation2d.fromDegrees(360);
        public static final double WRIST_REDUCTION = 56.842; // (9 / 1) * (4 / 1) * (93 / 57) * (90 / 93)
        public static final double KG_WRIST = 0.0; // This should remain zero based on construction of the arm
        public static final double KS_WRIST = 2.5;
        public static final double KV_WRIST = 0.0;
        public static final double KA_WRIST = 0.0;
        public static final double KP_WRIST = 100.0;
        public static final double KI_WRIST = 0.0;
        public static final double KD_WRIST = 0.0;
        public static final double WRIST_MAX_VELOCITY_RPS = 0.5;
        public static final double WRIST_MAX_ACCELERATION_RPS2 = 8.0;
        public static final double WRIST_MAX_JERK_RPS3 = 60.0;

        public static final double H_TOWER_GROUND_HEIGHT_METERS = Units.inchesToMeters(31.25);
        public static final double D_ARM_HORIZONTAL_OFFSET_METERS = Units.inchesToMeters(9.25);
        public static final double TOWER_CHASSIS_HEIGHT_METERS = Units.inchesToMeters(31.25 - 3.875);

        public static final Rotation2d SHOULDER_STARTING_OFFSET = Rotation2d.fromDegrees(-67);
        public static final Rotation2d ELBOW_STARTING_OFFSET = Rotation2d.fromDegrees(91);
        public static final Rotation2d WRIST_STARTING_OFFSET = Rotation2d.fromDegrees(90);

        public static final int LIMIT_SWITCH_ID = 8;

        /**
         * A class that stores all the states the arm could be at.
         * <br></br> <code> ArmStates.STATE; </code>
         */
        public class ArmStates {
                public static final ArmPose START = new ArmPose(
                                Units.inchesToMeters(19.75),
                                Units.inchesToMeters(34.5),
                                Rotation2d.fromDegrees(90));

                public static final ArmPose FRONT_FEEDER = new ArmPose(
                                Units.inchesToMeters(24),
                                Units.inchesToMeters(43),
                                Rotation2d.fromDegrees(15));

                public static final ArmPose FRONT_L1_REEF = new ArmPose(
                                Units.inchesToMeters(39.25),
                                Units.inchesToMeters(26.5),
                                Rotation2d.fromDegrees(-20));

                public static final ArmPose FRONT_L2_REEF = new ArmPose(
                                Units.inchesToMeters(25),
                                Units.inchesToMeters(36),
                                Rotation2d.fromDegrees(-15));

                //public static final ArmPose FRONT_L3_REEF = new ArmPose(
                                //Units.inchesToMeters(24),
                                //Units.inchesToMeters(57),
                                //Rotation2d.fromDegrees(-20));

                public static final ArmPose REAR_L3_REEF = new ArmPose(
                                Units.inchesToMeters(6),
                                Units.inchesToMeters(55),
                                Rotation2d.fromDegrees(200));

                public static final ArmPose REAR_L4_REEF = new ArmPose(
                                Units.inchesToMeters(10),
                                Units.inchesToMeters(78.5),
                                Rotation2d.fromDegrees(205));

        }
}
