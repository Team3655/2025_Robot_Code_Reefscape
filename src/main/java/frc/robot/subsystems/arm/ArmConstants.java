package frc.robot.subsystems.arm;

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
        // Used the check the validity of the arm setpoints only
        // MATT, FOR THE LOVE OF GOD MAKE SURE THIS ANGLE IS SMALL ENOUGH TO ACCOUNT FOR
        // BOTH THE CAD AND YOUR INITIAL OFFSET
        // ...
        // IF YOU CRASH THE CODE ON LAUNCH THIS IS WHY
        public static final Rotation2d SHOULDER_MIN_ANGLE = Rotation2d.fromDegrees(-80);
        public static final Rotation2d SHOULDER_MAX_ANGLE = Rotation2d.fromDegrees(90);
        public static final double SHOULDER_REDUCTION = 63; //63.7755
        public static final double KG_SHOULDER = 0.7;
        public static final double KS_SHOULDER = 3.0;
        public static final double KV_SHOULDER = 0.0;
        public static final double KA_SHOULDER = 0.0;
        public static final double KP_SHOULDER = 100.0;
        public static final double KI_SHOULDER = 0.0;
        public static final double KD_SHOULDER = 0.0;
        public static final double SHOULDER_MAX_VELOCITY_RPS = 0.3;
        public static final double SHOULDER_MAX_ACCELERATION_RPS2 = 0.1;
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
        public static final double KG_ELBOW = 0.95; 
        public static final double KS_ELBOW = 2.5;
        public static final double KV_ELBOW = 0.0;
        public static final double KA_ELBOW = 0.0;
        public static final double KP_ELBOW = 100;
        public static final double KI_ELBOW = 0.0;
        public static final double KD_ELBOW = 0.0;
        public static final double ELBOW_MAX_VELOCITY_RPS = 0.2;
        public static final double ELBOW_MAX_ACCELERATION_RPS2 = 0.1;
        public static final double ELBOW_MAX_JERK_RPS3 = 60.0;

        public static final int WRIST_MOTOR_ID = 36;
        public static final int WRIST_CANCODER_ID = 37;
        public static final Rotation2d WRIST_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(8);
        public static final Rotation2d ARM_DEFAULT_SETPOINT = Rotation2d.fromDegrees(75);
        public static final double WRIST_MASS_KG = 1;
        public static final Rotation2d WRIST_MIN_ANGLE_RADS = Rotation2d.fromDegrees(-360);
        public static final Rotation2d WRIST_MAX_ANGLE_RADS = Rotation2d.fromDegrees(360);
        public static final double WRIST_REDUCTION = 9 * (41/13);
        public static final double KG_WRIST = 0.0; // This should remain zero based on construction of the arm (changed to greater than 0 because change in construction of arm)
        public static final double KS_WRIST = 0.0;
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
        public static final Rotation2d WRIST_STARTING_OFFSET = Rotation2d.fromDegrees(0);

        public static final int LIMIT_SWITCH_ID = 8;

        /**
         * A class that stores all the states the arm could be at.
         * <br></br> <code> ArmStates.STATE; </code>
         */
        public class ArmStates {
                /**
                 * WARNING - Changing the initial/START pose of the robot arm
                 * to an invalid state could BREAK the robot.  Ensure the 
                 * setpoints are physically possible before deploying.
                 */
                public static final ArmPose START = new ArmPose(
                                Units.inchesToMeters(19.0), //cad number = 18.138
                                Units.inchesToMeters(34.9), //cad number = 35.059
                                Rotation2d.fromDegrees(0));

                public static final ArmPose FEEDER_START_TRANSITION = new ArmPose(
                                Units.inchesToMeters(20),
                                Units.inchesToMeters(41),
                                Rotation2d.fromDegrees(40));

                // public static final ArmPose TRANSITION = new ArmPose(
                //                 Units.inchesToMeters(22),
                //                 Units.inchesToMeters(47),
                //                 Rotation2d.fromDegrees(0));

                public static final ArmPose FRONT_FEEDER = new ArmPose(
                                Units.inchesToMeters(23.5),
                                Units.inchesToMeters(38),
                                Rotation2d.fromDegrees(30));

                public static final ArmPose FRONT_FEEDER_STRETCH = new ArmPose(
                                Units.inchesToMeters(28.5),
                                Units.inchesToMeters(39),
                                Rotation2d.fromDegrees(30));

                public static final ArmPose FRONT_L1_REEF = new ArmPose(
                                Units.inchesToMeters(36),
                                Units.inchesToMeters(29),
                                Rotation2d.fromDegrees(0));

                public static final ArmPose FRONT_L2_REEF = new ArmPose(
                                Units.inchesToMeters(23), //25
                                Units.inchesToMeters(34.5), //36
                                Rotation2d.fromDegrees(0)); //-10

                public static final ArmPose REAR_L3_REEF = new ArmPose(
                                Units.inchesToMeters(6),
                                Units.inchesToMeters(55.5),
                                Rotation2d.fromDegrees(205));

                public static final ArmPose REAR_L4_REEF = new ArmPose(
                                Units.inchesToMeters(6.5),
                                Units.inchesToMeters(78),
                                Rotation2d.fromDegrees(210));

                 public static final ArmPose ALGAE_STORE = new ArmPose(
                                Units.inchesToMeters(19),
                                Units.inchesToMeters(34.5),
                                Rotation2d.fromDegrees(160));

                public static final ArmPose FRONT_L1_ALGAE = new ArmPose(
                                Units.inchesToMeters(21), //34
                                Units.inchesToMeters(35.5), //42
                                Rotation2d.fromDegrees(115)); //100

                public static final ArmPose FRONT_L2_ALGAE = new ArmPose(
                                Units.inchesToMeters(29), //30
                                Units.inchesToMeters(39), //38
                                Rotation2d.fromDegrees(155)); //155
                
                public static final ArmPose FRONT_L2_ALGAE_ROTATED = new ArmPose(
                                Units.inchesToMeters(22), //34
                                Units.inchesToMeters(42), //59
                                Rotation2d.fromDegrees(160)); //185

                public static final ArmPose FRONT_BARGE = new ArmPose(
                                Units.inchesToMeters(11),
                                Units.inchesToMeters(78.5),
                                Rotation2d.fromDegrees(160));

                public static final ArmPose FRONT_BARGE_ROTATED = new ArmPose(
                                Units.inchesToMeters(11),
                                Units.inchesToMeters(78.5),
                                Rotation2d.fromDegrees(220));

        }
}
