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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmKinematics {

    private double d = 0.0;
    private double h = 0.0;

    private double L1 = 0.0;
    private double L2 = 0.0;
    private double L3 = 0.0;

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
    // private double L5 = 0.0;
    private double L6 = 0.0;

    private Rotation2d theta1 = Rotation2d.fromRadians(0.0);
    private Rotation2d relativeTheta2 = Rotation2d.fromRadians(0.0);
    private Rotation2d absoluteTheta2 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta3 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta4 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta6 = Rotation2d.fromRadians(0.0);

    private Rotation2d[] armAngles = new Rotation2d[2];

    /**
     * @param xSetpoint The x coordinate of the target in meters
     * @param ySetpoint The y coordinate of the target in meters
     * @return The angles of the arm joints - value is dependent on the active
     *         encoders.
     */
    public Rotation2d[] getArmAngles(double xTarget, double yTarget) {

        calculateInverseKinematics(xTarget, yTarget);

        // try {
        //     validateState(L4,
        //             L6,
        //             theta1.getRadians(),
        //             relativeTheta2.getRadians(),
        //             theta3.getRadians(),
        //             theta4.getRadians());
        // } catch (InvalidArmState e) {
        //     System.out.println(e.getMessage());
        //     throw e;
        // }

        armAngles[0] = theta1;

        switch (ArmConstants.activeEncoders) {
            case ABSOLUTE:
                armAngles[1] = absoluteTheta2;
                break;
            case RELATIVE:
                armAngles[1] = relativeTheta2;
                break;
        }

        SmartDashboard.putNumber("Theta1Deg", theta1.getDegrees());
        SmartDashboard.putNumber("Theta2Deg", armAngles[1].getDegrees());
        

        return armAngles;

    }

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

        double x = (elbow.getX() < 0) ? d - elbow.getX()
                : elbow.getX() + d;
        double y = elbow.getY();

        return new double[] { x, y };
    }

     //TODO: Garrett comment on these
    /**
     * Validates that the requested state of the arm is possible to achieve
     * 
     * @param theta
     * @param L4
     * @param L5
     * @param L6
     * @param theta1
     * @param relativeTheta2
     * @param theta3
     * @param thetaL4
     * @throws InvalidArmState Error to throw when state is not valid
     */
    private static void validateState(double L4, double L6, double theta1, double relativeTheta2,
            double theta3, double thetaL4) throws InvalidArmState {
        if (theta1 > (Math.PI / 2) || theta1 < -Units.degreesToRadians(-80)) {
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
}
