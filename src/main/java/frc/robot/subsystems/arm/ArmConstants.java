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

        public static final int ARM_CANDI_ID = 32;

        public static final int SHOULDER_MOTOR_ID = 30;
        public static final int SHOULDER_MOTOR_FOLLOWER_ID = 31;
        public static final Rotation2d SHOULDER_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double SHOULDER_LENGTH_METERS = Units.inchesToMeters(22.5);
        public static final double SHOULDER_MASS_KG = 1;
        // Used the check the validity of the arm setpoints only
        // MATT, FOR THE LOVE OF GOD MAKE SURE THIS ANGLE IS SMALL ENOUGH TO ACCOUNT FOR
        // BOTH THE CAD AND YOUR INITIAL OFFSET
        // ...
        // IF YOU CRASH THE CODE ON LAUNCH THIS IS WHY
        public static final Rotation2d SHOULDER_MIN_ANGLE = Rotation2d.fromDegrees(-80);
        public static final Rotation2d SHOULDER_MAX_ANGLE = Rotation2d.fromDegrees(90);
        public static final double SHOULDER_REDUCTION = 68.4; //63.7755
        public static final double KG_SHOULDER = 0.7;
        public static final double KS_SHOULDER = 3.25;
        public static final double KV_SHOULDER = 0.0;
        public static final double KA_SHOULDER = 0.0;
        public static final double KP_SHOULDER = 100.0;
        public static final double KI_SHOULDER = 0.0;
        public static final double KD_SHOULDER = 0.0;
        public static final double SHOULDER_MAX_VELOCITY_RPS = 0.4; //0.3
        public static final double SHOULDER_MAX_ACCELERATION_RPS2 = 0.1; //0.1
        public static final double SHOULDER_MAX_JERK_RPS3 = 60.0;

        public static final int ELBOW_MOTOR_ID = 33;
        public static final int ELBOW_MOTOR_FOLLOWER_ID = 34;
        public static final Rotation2d ELBOW_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double ELBOW_LENGTH_METERS = Units.inchesToMeters(24.25);
        public static final double ELBOW_MASS_KG = 1;
        public static final Rotation2d ELBOW_MIN_ANGLE_RADS = Rotation2d.fromDegrees(0);
        public static final Rotation2d ELBOW_MAX_ANGLE_RADS = Rotation2d.fromDegrees(180);
        public static final double ELBOW_REDUCTION = 68.4;
        public static final double KG_ELBOW = 0.95; 
        public static final double KS_ELBOW = 2.75;
        public static final double KV_ELBOW = 0.0;
        public static final double KA_ELBOW = 0.0;
        public static final double KP_ELBOW = 100;
        public static final double KI_ELBOW = 0.0;
        public static final double KD_ELBOW = 0.0;
        public static final double ELBOW_MAX_VELOCITY_RPS = 0.2; //0.2
        public static final double ELBOW_MAX_ACCELERATION_RPS2 = 0.1; //0.1
        public static final double ELBOW_MAX_JERK_RPS3 = 60.0;

        public static final int WRIST_MOTOR_ID = 36;
        public static final Rotation2d WRIST_ENCODER_OFFSET = Rotation2d.fromRotations(0);
        public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(8);
        public static final Rotation2d ARM_DEFAULT_SETPOINT = Rotation2d.fromDegrees(75);
        public static final double WRIST_MASS_KG = 1;
        public static final Rotation2d WRIST_MIN_ANGLE_RADS = Rotation2d.fromDegrees(-20);
        public static final Rotation2d WRIST_MAX_ANGLE_RADS = Rotation2d.fromDegrees(340);
        public static final double WRIST_REDUCTION = 3 * 9 * (41/13);
        public static final double KG_WRIST = 0.0; // This should remain zero based on construction of the arm (changed to greater than 0 because change in construction of arm)
        public static final double KS_WRIST = 0.5;
        public static final double KV_WRIST = 0.0;
        public static final double KA_WRIST = 0.0;
        public static final double KP_WRIST = 300.0;
        public static final double KI_WRIST = 0.0;
        public static final double KD_WRIST = 0.0;
        public static final double WRIST_MAX_VELOCITY_RPS = 2.5; //1.5
        public static final double WRIST_MAX_ACCELERATION_RPS2 = 8.0;
        public static final double WRIST_MAX_JERK_RPS3 = 60.0;

        public static final double H_TOWER_GROUND_HEIGHT_METERS = Units.inchesToMeters(31.25);
        public static final double D_ARM_HORIZONTAL_OFFSET_METERS = Units.inchesToMeters(9.25);
        public static final double TOWER_CHASSIS_HEIGHT_METERS = Units.inchesToMeters(31.25 - 3.875);

        public static final Rotation2d SHOULDER_STARTING_OFFSET = Rotation2d.fromDegrees(-67);
        public static final Rotation2d ELBOW_STARTING_OFFSET = Rotation2d.fromDegrees(91);
        public static final Rotation2d WRIST_STARTING_OFFSET = Rotation2d.fromDegrees(0);

        public static final int LIMIT_SWITCH_ID = 8;

        public enum ArmPoseNames {
                START,
                FEEDER_START_TRANSITION,
                FRONT_FEEDER,
                FRONT_FEEDER_STRETCH,
                FRONT_L1_REEF,
                FRONT_L2_REEF,
                REAR_L3_REEF,
                REAR_L4_REEF_TRANSITION,
                REAR_L4_REEF_WRIST_FLIP,
                REAR_L4_REEF,
                FLOOR_ALGAE,
                PREP_L1_ALGAE,
                PULL_L1_ALGAE,
                PREP_L2_ALGAE,
                PULL_L2_ALGAE,
                PROCESSOR,
                BARGE_PREP,
                BARGE_THROW,
                CLIMB_STRETCH
        }

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

                 //Stored Positions
                public static final ArmPose START = new ArmPose(
                                Units.inchesToMeters(19.0), //cad number = 18.138
                                Units.inchesToMeters(34.9), //cad number = 35.059
                                Rotation2d.fromDegrees(0),
                                ArmPoseNames.START);

                //Feeder Station Positions
                public static final ArmPose FEEDER_START_TRANSITION = new ArmPose(
                                Units.inchesToMeters(20),
                                Units.inchesToMeters(41),
                                Rotation2d.fromDegrees(40),
                                ArmPoseNames.FEEDER_START_TRANSITION);

                public static final ArmPose FRONT_FEEDER = new ArmPose(
                                Units.inchesToMeters(24), //Actual 25.5
                                Units.inchesToMeters(39), //Actual 39
                                Rotation2d.fromDegrees(30),
                                ArmPoseNames.FRONT_FEEDER);

                public static final ArmPose FRONT_FEEDER_STRETCH = new ArmPose(
                                Units.inchesToMeters(28.5),
                                Units.inchesToMeters(39),
                                Rotation2d.fromDegrees(30),
                                ArmPoseNames.FRONT_FEEDER_STRETCH);

                //Coral Postitions
                public static final ArmPose FRONT_L1_REEF = new ArmPose(
                                Units.inchesToMeters(36),
                                Units.inchesToMeters(29),
                                Rotation2d.fromDegrees(0),
                                ArmPoseNames.FRONT_L1_REEF);

                public static final ArmPose FRONT_L2_REEF = new ArmPose(
                                Units.inchesToMeters(25),
                                Units.inchesToMeters(34.5),
                                Rotation2d.fromDegrees(0),
                                ArmPoseNames.FRONT_L2_REEF);

                public static final ArmPose REAR_L3_REEF = new ArmPose(
                                Units.inchesToMeters(6),
                                Units.inchesToMeters(55.5),
                                Rotation2d.fromDegrees(213),
                                ArmPoseNames.REAR_L3_REEF);

                public static final ArmPose REAR_L4_REEF_TRANSITION = new ArmPose(
                                Units.inchesToMeters(20),
                                Units.inchesToMeters(60),
                                Rotation2d.fromDegrees(0),
                                ArmPoseNames.REAR_L4_REEF_TRANSITION);

                public static final ArmPose REAR_L4_REEF_WRIST_FLIP = new ArmPose(
                                Units.inchesToMeters(5.5),
                                Units.inchesToMeters(77),
                                Rotation2d.fromDegrees(90),
                                ArmPoseNames.REAR_L4_REEF_WRIST_FLIP);

                public static final ArmPose REAR_L4_REEF = new ArmPose(
                                Units.inchesToMeters(5.5), //6
                                Units.inchesToMeters(77), //77
                                Rotation2d.fromDegrees(230), //230
                                ArmPoseNames.REAR_L4_REEF);

                //Algae Positions
                public static final ArmPose FLOOR_ALGAE = new ArmPose(
                                Units.inchesToMeters(40), //39.5
                                Units.inchesToMeters(21), //21.75
                                Rotation2d.fromDegrees(-70),
                                ArmPoseNames.FLOOR_ALGAE);

                public static final ArmPose PREP_L1_ALGAE = new ArmPose(
                                Units.inchesToMeters(26),
                                Units.inchesToMeters(39),
                                Rotation2d.fromDegrees(0),
                                ArmPoseNames.PREP_L1_ALGAE);

                public static final ArmPose PULL_L1_ALGAE = new ArmPose(
                                Units.inchesToMeters(26),
                                Units.inchesToMeters(39),
                                Rotation2d.fromDegrees(0),
                                ArmPoseNames.PULL_L1_ALGAE);

                public static final ArmPose PREP_L2_ALGAE = new ArmPose(
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(52),
                                Rotation2d.fromDegrees(200),
                                ArmPoseNames.PREP_L2_ALGAE);
        
                public static final ArmPose PULL_L2_ALGAE = new ArmPose(
                                Units.inchesToMeters(10),
                                Units.inchesToMeters(52),
                                Rotation2d.fromDegrees(180),
                                ArmPoseNames.PULL_L2_ALGAE);

                public static final ArmPose PROCESSOR = new ArmPose(
                                Units.inchesToMeters(36), 
                                Units.inchesToMeters(29),
                                Rotation2d.fromDegrees(-30),
                                ArmPoseNames.PROCESSOR);

                public static final ArmPose BARGE_PREP = new ArmPose(
                                Units.inchesToMeters(9),
                                Units.inchesToMeters(77),
                                Rotation2d.fromDegrees(-30),
                                ArmPoseNames.BARGE_PREP);
                
                public static final ArmPose BARGE_THROW = new ArmPose(
                                Units.inchesToMeters(9),
                                Units.inchesToMeters(77),
                                Rotation2d.fromDegrees(100),
                                ArmPoseNames.BARGE_THROW);

                //Climb Positions
                public static final ArmPose CLIMB_STRETCH = new ArmPose(
                                Units.inchesToMeters(34), 
                                Units.inchesToMeters(28),
                                Rotation2d.fromDegrees(100),
                                ArmPoseNames.CLIMB_STRETCH);
        }
}
