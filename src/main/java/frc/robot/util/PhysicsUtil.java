package frc.robot.util;

public class PhysicsUtil {
  
  /**
   * Estimates MOI of the robot in kilogram meter squared (kg·m²)
   * @param mass The mass of the robot in KG
   * @param widthX The width of the robot from bumper to bumper in X
   * @param widthY The width of the robot from bumper to bumper in X
   * @return The estimated MOI
   */
  public static double estimateRobotMOI(double mass, double widthX, double widthY) {
    return (1.0 / 12.0) * mass * (Math.pow(widthX, 2) + Math.pow(widthY, 2));
  }

}
