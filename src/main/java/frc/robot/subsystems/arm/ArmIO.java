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
    public double shoulderVelocityRadPerSec = 0.0;
    public double shoulderAppliedVolts = 0.0;
    public double[] shoulderCurrentAmps = new double[] {};

    public Rotation2d elbowPosition = new Rotation2d();
    public double elbowVelocityRadPerSec = 0.0;
    public double elbowAppliedVolts = 0.0;
    public double[] elbowCurrentAmps = new double[] {};

    public Rotation2d wristPosition = new Rotation2d();
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double[] wristCurrentAmps = new double[] {};

  }

  public default void updateInputs(ArmIOInputs inputs) {}
  public default void setShoulderPosition(Rotation2d position) {}
  public default void setElbowPosition(Rotation2d position) {}
  public default void setWristPosition(Rotation2d position) {}
  public default void setShoulderVoltage(double volts) {}
  public default void setElbowVoltage(double volts) {}
  public default void setWristVoltage(double volts) {}
}
