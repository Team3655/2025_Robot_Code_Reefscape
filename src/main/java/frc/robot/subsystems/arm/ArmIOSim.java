package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {

  private final DCMotor gearbox = DCMotor.getKrakenX60(1);

  private double shoulderVolts = 0.0;
  private double elbowVolts = 0.0;
  private double wristVolts = 0.0;

  // Physics simulation for the shoulder
  private final SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
      gearbox,
      ArmConstants.SHOULDER_REDUCTION,
      SingleJointedArmSim.estimateMOI(ArmConstants.SHOULDER_LENGTH_METERS,
          ArmConstants.SHOULDER_MASS_KG),
      ArmConstants.SHOULDER_LENGTH_METERS,
      Rotation2d.fromDegrees(-360).getRadians(),
      Rotation2d.fromDegrees(360).getRadians(),
      false,
      Rotation2d.fromDegrees(0).getRadians());

  // Physics simulation for the elbow
  private final SingleJointedArmSim elbowSim = new SingleJointedArmSim(
      gearbox,
      ArmConstants.ELBOW_REDUCTION,
      SingleJointedArmSim.estimateMOI(ArmConstants.ELBOW_LENGTH_METERS,
          ArmConstants.ELBOW_MASS_KG),
      ArmConstants.ELBOW_LENGTH_METERS,
      Rotation2d.fromDegrees(-360).getRadians(),
      Rotation2d.fromDegrees(360).getRadians(),
      false,
      Rotation2d.fromDegrees(0).getRadians());

  // Physics simulation for the wrist
  private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
      gearbox,
      ArmConstants.WRIST_REDUCTION,
      SingleJointedArmSim.estimateMOI(ArmConstants.WRIST_LENGTH_METERS,
          ArmConstants.WRIST_MASS_KG),
      ArmConstants.WRIST_LENGTH_METERS,
      ArmConstants.WRIST_MIN_ANGLE_RADS.getRadians(),
      ArmConstants.WRIST_MAX_ANGLE_RADS.getRadians(),
      false,
      Rotation2d.fromDegrees(0).getRadians());

  private final PIDController shoulderController = new PIDController(10, ArmConstants.KI_SHOULDER,
      ArmConstants.KD_SHOULDER);
  private final PIDController elbowController = new PIDController(10, ArmConstants.KI_ELBOW, ArmConstants.KD_ELBOW);
  private final PIDController wristController = new PIDController(ArmConstants.KP_WRIST, ArmConstants.KI_WRIST,
      ArmConstants.KD_WRIST);

  /**
   * Updates the inputs created in ArmIO to the data calculated from the simulated
   * motors.
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {

    shoulderVolts = shoulderController.calculate(shoulderSim.getAngleRads());
    elbowVolts = elbowController.calculate(elbowSim.getAngleRads());
    wristVolts = wristController.calculate(wristSim.getAngleRads());

    shoulderSim.setInputVoltage(shoulderVolts);
    elbowSim.setInputVoltage(elbowVolts);
    wristSim.setInputVoltage(wristVolts);

    shoulderSim.update(0.02);
    elbowSim.update(0.02);
    wristSim.update(0.02);

    inputs.shoulderPosition = Rotation2d.fromRadians(shoulderSim.getAngleRads());
    inputs.shoulderCurrentAmps = new double[] { shoulderSim.getCurrentDrawAmps() };
    inputs.shoulderVelocityRadPerSec = shoulderSim.getVelocityRadPerSec();

    inputs.elbowPosition = Rotation2d.fromRadians(elbowSim.getAngleRads());
    inputs.elbowCurrentAmps = new double[] { elbowSim.getCurrentDrawAmps() };
    inputs.elbowVelocityRadPerSec = elbowSim.getVelocityRadPerSec();

    inputs.wristPosition = Rotation2d.fromRadians(wristSim.getAngleRads());
    inputs.wristCurrentAmps = new double[] { wristSim.getCurrentDrawAmps() };
    inputs.wristVelocityRadPerSec = wristSim.getVelocityRadPerSec();
  }

  /**
   * Sets the shoulder voltage, clamps the input value between 12 and -12
   * 
   * @param volts the volts to set
   */
  @Override
  public void setShoulderVoltage(double volts) {
    shoulderVolts = MathUtil.clamp(volts, -12, 12);
  }

  /**
   * Sets the elbow voltage, clamps the input value between 12 and -12
   * 
   * @param volts the volts to set
   */
  @Override
  public void setElbowVoltage(double volts) {
    elbowVolts = MathUtil.clamp(volts, -12, 12);
  }

  /**
   * Sets the shoulder voltage, clamps the input value between 12 and -12
   * 
   * @param volts the volts to set
   */
  @Override
  public void setWristVoltage(double volts) {
    wristVolts = MathUtil.clamp(volts, -12, 12);
  }

  /**
   * Sets the position of the shoulder PID controller
   * 
   * @param position A Rotation2d to set the position
   */
  @Override
  public void setShoulderPosition(Rotation2d position) {
    shoulderController.setSetpoint(position.getRadians());
  }

  /**
   * Sets the position of the elbow PID controller
   * 
   * @param position A Rotation2d to set the position
   */
  @Override
  public void setElbowPosition(Rotation2d position) {
    elbowController.setSetpoint(position.getRadians());
  }

  /**
   * Sets the position of the wrist PID controller
   * 
   * @param position A Rotation2d to set the position
   */
  @Override
  public void setWristPosition(Rotation2d position) {
    wristController.setSetpoint(position.getRadians());
  }
}
