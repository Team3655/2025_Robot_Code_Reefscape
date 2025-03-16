package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {

  /**
   * This class holds all the data that is logged from the arm.
   */
  @AutoLog
  public class ArmIOInputs {

    public Rotation2d shoulderPosition = new Rotation2d();
    public double[] shoulderCurrentAmps = new double[] {};

    public Rotation2d elbowPosition = new Rotation2d();
    public double[] elbowCurrentAmps = new double[] {};

    public Rotation2d wristPosition = new Rotation2d();
    public double[] wristCurrentAmps = new double[] {};

  }

  public default void updateInputs(ArmIOInputs inputs) {}
  public default void setShoulderPosition(Rotation2d position) {}
  public default void setElbowPosition(Rotation2d position) {}
  public default void setWristPosition(Rotation2d position) {}
}
