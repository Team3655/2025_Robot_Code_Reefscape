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

public class ArmKinematics {

    // https://cad.onshape.com/documents/0eb11a58606ee3c3dda8aa0d/w/d1c684d1c568543878764fb7/e/4c2980432bfd825f337a321f?renderMode=0&uiState=678c52c70a7cb65a2aa773cc
    
    private double xTarget = 0.0;
    private double yTarget = 0.0;

    private double d = ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS;
    private double h = ArmConstants.H_TOWER_GROUND_HEIGHT_METERS;

    private double L1 = ArmConstants.TOWER_CHASSIS_HEIGHT_METERS;
    private double L2 = ArmConstants.SHOULDER_LENGTH_METERS;
    private double L3 = ArmConstants.ELBOW_LENGTH_METERS;
    private double L4 = 0.0;
    private double L5 = 0.0;
    private double L6 = 0.0;

    private Rotation2d theta1 = Rotation2d.fromRadians(0.0);
    private Rotation2d relativeTheta2 = Rotation2d.fromRadians(0.0);
    private Rotation2d absoluteTheta2 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta3 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta4 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta6 = Rotation2d.fromRadians(0.0);
    private Rotation2d theta7 = Rotation2d.fromRadians(0.0);

    private Rotation2d[] armAngles = new Rotation2d[2];

    /**
     * @param xSetpoint The x coordinate of the target
     * @param ySetpoint The y coordinate of the target
     * @return The angles of the arm joints - value is dependent on the active encoders
     */
    public Rotation2d[] calculateArmAngles(double xTarget, double yTarget) {
        this.xTarget = xTarget;
        this.yTarget = yTarget;

        calculateMeasurements();

        armAngles[0] = theta1;
        switch(ArmConstants.activeEncoders) {
            case ABSOLUTE:
                armAngles[1] = absoluteTheta2;
                break;
            case RELATIVE:
                armAngles[1] = relativeTheta2;
            break;
        }

        return armAngles;

    }

    private void calculateMeasurements() {
        // Derived from arm constants and setpoint - Pythagorean Theorem
        L4 = Math.sqrt(
            (Math.pow(xTarget - ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS, 2)
                + Math.pow(yTarget - ArmConstants.H_TOWER_GROUND_HEIGHT_METERS, 2)));

        // Derived from arm constants and setpoint - Pythagorean Theorem
        L6 = Math.sqrt(
            Math.pow(xTarget - ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS, 2) + Math.pow(
                yTarget - ArmConstants.H_TOWER_GROUND_HEIGHT_METERS + ArmConstants.TOWER_CHASSIS_HEIGHT_METERS,
                2));

        // Derived from arm constants and L4 - Law of Cosines
        theta3 = Rotation2d.fromRadians(
            Math.acos(
                (Math.pow(L4, 2) + Math.pow(ArmConstants.SHOULDER_LENGTH_METERS, 2)
                    - Math.pow(ArmConstants.ELBOW_LENGTH_METERS, 2)) /
                (2 * L4 * ArmConstants.SHOULDER_LENGTH_METERS)));

        // Derived from arm constants and L4 - Law of Cosines
        theta4 = Rotation2d.fromRadians(Math.acos(
            (Math.pow(ArmConstants.SHOULDER_LENGTH_METERS, 2) + Math.pow(ArmConstants.ELBOW_LENGTH_METERS, 2)
                - Math.pow(L4, 2)) /
                (2 * ArmConstants.SHOULDER_LENGTH_METERS * ArmConstants.ELBOW_LENGTH_METERS)));

        // Absolute angle of L3 joint is supplement to theta4 - Definition of Supplemental Angles
        absoluteTheta2 = Rotation2d.fromRadians(Math.PI).minus(theta4);

        theta7 = Rotation2d.fromRadians(L1 * (Math.sin(theta6)) / L6);
    }

    private boolean isBelowHorizontal() {
        return true;
    }

    private boolean isBehindRobot() {
        
        boolean isBehind = xTarget < ArmConstants.D_ARM_HORIZONTAL_OFFSET_METERS;

        return isBehind;
    }
    
}
