/**
 * 
 * OnShape: https://cad.onshape.com/documents/0eb11a58606ee3c3dda8aa0d/w/d1c684d1c568543878764fb7/e/4c2980432bfd825f337a321f?renderMode=0&uiState=678c52c70a7cb65a2aa773cc
 * 
 * The robot operates on three segments: TOWER_CHASSIS_HEIGHT_METERS, SHOULDER_LENGTH_METERS, and ELBOW_LENGTH_METERS
 * Although only the final two move, the origin of the coordinate system sits on the ground at the back
 * of the robot.
 * 
 * When running on relative encoders within the robot, the final segment (ELBOW_LENGTH_METERS) operates on an angle relative 
 * to the horizontal/Earth
 * 
 * Absoulte encoders are fixed to the previous segment.  Measuring with absolute, the final segment (ELBOW_LENGTH_METERS)
 * operates at an angle relative to second segment (SHOULDER_LENGTH_METERS)
 */

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.arm.ArmConstants.ArmEncoders;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

/**
 * Kinematics class for a two stage arm
 */
public class ArmKinematics {

    private double d = 0.0;
    private double h = 0.0;

    private double L1 = 0.0;
    private double L2 = 0.0;
    private double L3 = 0.0;

    Notification invalidArmStateNotification = new Notification(
        NotificationLevel.ERROR, 
        "INVALID ARM STATE", 
        "x and y setpoints are resulting in invalid arm angles.  Check your calculations");

    /**
     * Creates a new `ArmKinematics` object
     * 
     * @param d  How far the arm is from the back of the robot
     * @param h  How far from the ground the first pivot point is
     * @param L1 Length of the "tower" that the arm rests on
     * @param L2 Length of the first stage of the arm. (shoulder)
     * @param L3 Length of the second stage of the arm. (elbow)
     */
    public ArmKinematics(double d, double h, double L1, double L2, double L3) {
        this.d = d;
        this.h = h;
        this.L1 = L1;
        this.L2 = L2;
        this.L3 = L3;
    }

    private double L4 = 0.0;
    private double L6 = 0.0;

    private Rotation2d theta1 = Rotation2d.fromRadians(0.0);
    private Rotation2d relativeTheta2 = Rotation2d.fromRadians(0.0);
    private Rotation2d absoluteTheta2 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta3 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta4 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta6 = Rotation2d.fromRadians(0.0);

    // Ensure that these values are always updated
    public Rotation2d[] currentArmAngles = new Rotation2d[2];
    private Rotation2d[] calculatedArmAngles = new Rotation2d[2];

    /**
     * Gets the arm angles from the calculations
     * 
     * @param xSetpoint The x coordinate of the target in meters
     * @param ySetpoint The y coordinate of the target in meters
     * @return The angles of the arm joints - value is dependent on the active encoders.
     */
    public Rotation2d[] getArmAngles(double xTarget, double yTarget, ArmEncoders encoderType) {

        calculateInverseKinematics(xTarget, yTarget);

        try {
            validateState(L4,
                    L6,
                    theta1.getRadians(),
                    relativeTheta2.getRadians(),
                    theta3.getRadians(),
                    theta4.getRadians());

            calculatedArmAngles[0] = theta1;

            switch (encoderType) {
                case ABSOLUTE:

                    calculatedArmAngles[1] = absoluteTheta2;
                    break;
                case RELATIVE:

                    calculatedArmAngles[1] = relativeTheta2;
                    break;
                default:

                    DriverStation.reportError("INVALID ARM ENCODER TYPE", false);
                    break;
            }

            return calculatedArmAngles;

        } catch (InvalidArmState e) {
            DriverStation.reportError("INVALID ARM STATE INPUT, CANNOT MOVE ARM", true);
            Elastic.sendNotification(invalidArmStateNotification);
            return currentArmAngles;
        }

    }

    /**
     * Calculates arm angles using inverse kinematics
     * 
     * @param xTarget The x position in meters to set the arm at
     * @param yTarget The y position in meters to set the arm at
     */
    private void calculateInverseKinematics(double xTarget, double yTarget) {
        // Derived from arm constants and setpoint - Pythagorean Theorem
        L4 = Math.sqrt(
                (Math.pow(xTarget - d, 2)
                        + Math.pow(yTarget - h, 2)));

        // Derived from arm constants and setpoint - Pythagorean Theorem
        L6 = Math.sqrt(
                Math.pow(xTarget - d, 2) + Math.pow(
                        yTarget - h + L1,
                        2));

        // Derived from arm constants and L4 - Law of Cosines
        theta3 = Rotation2d.fromRadians(
                Math.acos(
                        (Math.pow(L4, 2) + Math.pow(L2, 2)
                                - Math.pow(L3, 2)) /
                                (2 * L4 * L2)));

        // Derived from arm constants and L4 - Law of Cosines
        theta4 = Rotation2d.fromRadians(Math.acos(
                (Math.pow(L2, 2) + Math.pow(L3, 2)
                        - Math.pow(L4, 2)) /
                        (2 * L2 * L3)));

        calculateTheta6(xTarget, yTarget);

        // Derived from theta6 and theta3 - Angle Addition Postulate
        theta1 = theta6.minus(theta3).minus(Rotation2d.kCCW_Pi_2);

        // Absolute angle of L3 joint is supplement to theta4 - Definition of
        // Supplemental Angles
        // Are .kPi and .fromRations(MATH.PI) the same?
        absoluteTheta2 = Rotation2d.fromRadians(Math.PI).minus(theta4);

        // Derived from theta1 and theta4 - Supplementary angles, Triangle Sum Theorem,
        // Corresponding Angles Postulate
        relativeTheta2 = Rotation2d.kPi.plus(theta1).minus(theta4);

    }

    /**
     * 
     * @param xTarget
     * @param yTarget
     */
    private void calculateTheta6(double xTarget, double yTarget) {
        // Triangles used to calculate measurements change when moving the target behind
        // the tower (L1)
        if (xTarget < d) {
            // Derived from arm constants, L4, and L6 - Law of Cosines
            theta6 = Rotation2d.fromRadians(
                    (2 * Math.PI) -
                            (Math.acos(
                                    (Math.pow(L1, 2) + Math.pow(L4, 2) - Math.pow(L6, 2)) /
                                            (2 * L1 * L4))));
        } else if (xTarget == d) {
            theta6 = Rotation2d.kPi;
        } else {
            // Derived from arm constants, L4, and L6 - Law of Cosines
            theta6 = Rotation2d.fromRadians(
                    Math.acos(
                            (Math.pow(L1, 2) + Math.pow(L4, 2) - Math.pow(L6, 2)) /
                                    (2 * L1 * L4)));
        }
    }

    public double[] calculateForwardKinematics(Rotation2d theta1, Rotation2d theta2) {

        Translation2d base = new Translation2d(h, Rotation2d.fromDegrees(90));
        Translation2d shoulder = base.plus(new Translation2d(L1, theta1));
        Translation2d elbow = shoulder.plus(new Translation2d(L2, theta2));

        double x = (elbow.getX() < 0) ? d - elbow.getX() : elbow.getX() + d;
        double y = elbow.getY();

        return new double[] { x, y };
    }

    /**
     * Validates that the requested state of the arm is possible to achieve
     * 
     * @param L4             Line segment between first joint and end of arm
     * @param L5             Line segment between base of tower to second joint
     * @param L6             Line segment between base of tower and end of arm
     * @param theta1         Angle of the shoulder relative to the horizontal
     * @param relativeTheta2 Angle of the elbow relative to the Earth
     * @param theta3         Angle between shoulder and L4
     * @param thetaL4        Obtuse angle between shoulder and elbow - across from
     *                       L4
     * @throws InvalidArmState Error to throw when state is not valid
     */
    private static void validateState(double L4, double L6, double theta1, double relativeTheta2,
            double theta3, double thetaL4) throws InvalidArmState {
        if (theta1 > (Math.PI / 2) || theta1 < Units.degreesToRadians(-80)) {
            throw new InvalidArmState("ARM SEGMENT 2 CANNOT EXTEND PAST 180 DEG.  THETA1:  " + theta1);
        }

        double[] values = new double[6];
        values[0] = L4;
        values[1] = L6;
        values[2] = theta1;
        values[3] = relativeTheta2;
        values[4] = theta3;
        values[5] = thetaL4;

        for (int i = 0; i < 6; i++) {
            if (!Double.isFinite(values[i])) {
                throw new InvalidArmState("ARM OUT OF BOUNDS - INVALID X AND Y");
            }
        }
    }

    /**
     * Error to throw when state is not valid
     */
    private static class InvalidArmState extends RuntimeException {
        public InvalidArmState(String m) {
            super(m);
        }
    }


    /**
     * Validates that the requested bump is possible to achieve
     * WARNING: THIS DOES NOT ACCOUNT FOR BUMPS THAT RUN INTO THE CHASSIS OR THE TOWER
     * USE ONLY TO VALIDATE UPPER CIRCLE BUMPS
     * @param xTarget The requested x position in meters
     * @param yTarget The requested y position in meters
     * @param encoderType
     * @return
     */
    public boolean isValidBumpRequest(double xTarget, double yTarget) {
      double h = ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS;
      double k = ArmConstants.H_TOWER_GROUND_HEIGHT_METERS;
      double r = ArmConstants.SHOULDER_LENGTH_METERS + ArmConstants.ELBOW_LENGTH_METERS;

      /**
       * Equation for a circle
       * Center (h, k) is at the shoulder pivot point
       * Radius, r, is defined by the length of the shoulder and elbow combined
       */
      boolean insideArmCircle = Math.pow(xTarget - h, 2) + Math.pow(yTarget - k, 2) <= Math.pow(r, 2);

      return insideArmCircle;
    }

    /**
     * Calculates the y position of the elbow given the x position of the bump
     * Y setpoint will always be positive - dictated by the coordinate system of the robot
     * @param bumpX The x position of the bump in meters
     * @return
     */
    public double calculateArcYSetpoint(double bumpX, double currentXTarget, double currentYTarget) {

      L4 = Math.sqrt(
        (Math.pow(currentXTarget - d, 2)
                + Math.pow(currentYTarget - h, 2)));

      double r = L4;
      double h = ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS;
      double k = ArmConstants.H_TOWER_GROUND_HEIGHT_METERS;

      // Pythagorean Theorem given leg x and hypotenuse r
      // Equation of a circle: (x-h)^2 + (y-k)^2 = r^2 where (h, k) is the center of the circle
      double bumpY = Math.sqrt(Math.pow(r, 2) - Math.pow(bumpX - h, 2)) + k;

      return bumpY;
    }

    public double calculateArcXSetpoint(double bumpY) {
      double r = ArmConstants.SHOULDER_LENGTH_METERS + ArmConstants.ELBOW_LENGTH_METERS;
      double h = ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS;
      double k = ArmConstants.H_TOWER_GROUND_HEIGHT_METERS;

      // Pythagorean Theorem given leg y and hypotenuse r
      double bumpX = Math.sqrt(Math.pow(r, 2) - Math.pow(bumpY - k, 2)) + h;

      return bumpX;
    }
}
