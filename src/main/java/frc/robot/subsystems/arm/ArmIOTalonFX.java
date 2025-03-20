package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {

  private final TalonFX shoulderLeaderTalon;
  private final TalonFX shoulderFollowerTalon;
  private final TalonFX elbowLeaderTalon;
  private final TalonFX elbowFollowerTalon;
  private final TalonFX wristTalon;

  private final StatusSignal<Angle> shoulderPosition;
  private final StatusSignal<Voltage> shoulderAppliedVolts;
  private final StatusSignal<Current> shoulderCurrent;

  private final StatusSignal<Angle> elbowPosition;
  private final StatusSignal<Voltage> elbowAppliedVolts;
  private final StatusSignal<Current> elbowCurrent;

  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<Voltage> wristAppliedVolts;
  private final StatusSignal<Current> wristCurrent;

  private final TalonFXConfiguration shoulderLeaderConfig;
  private final TalonFXConfiguration elbowConfiguration;
  private final TalonFXConfiguration wristConfiguration;

  public ArmIOTalonFX() {
    shoulderLeaderTalon = new TalonFX(ArmConstants.SHOULDER_MOTOR_ID, Constants.CANIVORE_NAME);
    shoulderFollowerTalon = new TalonFX(ArmConstants.SHOULDER_MOTOR_FOLLOWER_ID, Constants.CANIVORE_NAME);
    elbowLeaderTalon = new TalonFX(ArmConstants.ELBOW_MOTOR_ID, Constants.CANIVORE_NAME);
    elbowFollowerTalon = new TalonFX(ArmConstants.ELBOW_MOTOR_FOLLOWER_ID, Constants.CANIVORE_NAME);
    wristTalon = new TalonFX(ArmConstants.WRIST_MOTOR_ID, "rio");

    shoulderLeaderConfig = new TalonFXConfiguration();
    elbowConfiguration = new TalonFXConfiguration();
    wristConfiguration = new TalonFXConfiguration();

    switch (ArmConstants.activeEncoders) {
      case ABSOLUTE:

        break;
      case RELATIVE:

        // Use the motor's internal sensor
        shoulderLeaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        elbowConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        wristConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        shoulderLeaderConfig.Feedback.SensorToMechanismRatio = ArmConstants.SHOULDER_REDUCTION;
        elbowConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.ELBOW_REDUCTION;
        wristConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.WRIST_REDUCTION;

        shoulderLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elbowConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set soft limts and enable them
        // shoulderLeaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.SHOULDER_MAX_ANGLE_RADS.getDegrees();
        // shoulderLeaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.SHOULDER_MIN_ANGLE_RADS.getDegrees();
        // shoulderLeaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // shoulderLeaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // wristConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.WRIST_MIN_ANGLE_RADS.getRotations();
        // wristConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // wristConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.WRIST_MAX_ANGLE_RADS.getRotations();
        // wristConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        break;
      default:

        // Default to relative
        shoulderLeaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        elbowConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        wristConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        shoulderLeaderConfig.Feedback.SensorToMechanismRatio = ArmConstants.SHOULDER_REDUCTION;
        elbowConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.ELBOW_REDUCTION;
        wristConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.WRIST_REDUCTION;

        shoulderLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elbowConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        break;

    }

    // Set PID values for each stage of the arm
    var slot0Shoulder = shoulderLeaderConfig.Slot0;
    slot0Shoulder.kG = ArmConstants.KG_SHOULDER;
    slot0Shoulder.kS = ArmConstants.KS_SHOULDER;
    slot0Shoulder.kV = ArmConstants.KV_SHOULDER;
    slot0Shoulder.kA = ArmConstants.KA_SHOULDER;
    slot0Shoulder.kP = ArmConstants.KP_SHOULDER;
    slot0Shoulder.kI = ArmConstants.KI_SHOULDER;
    slot0Shoulder.kD = ArmConstants.KD_SHOULDER;

    var slot0Elbow = elbowConfiguration.Slot0;
    slot0Elbow.kG = ArmConstants.KG_ELBOW;
    slot0Elbow.kS = ArmConstants.KS_ELBOW;
    slot0Elbow.kV = ArmConstants.KV_ELBOW;
    slot0Elbow.kA = ArmConstants.KA_ELBOW;
    slot0Elbow.kP = ArmConstants.KP_ELBOW;
    slot0Elbow.kI = ArmConstants.KI_ELBOW;
    slot0Elbow.kD = ArmConstants.KD_ELBOW;

    var slot0Wrist = wristConfiguration.Slot0;
    slot0Wrist.kG = ArmConstants.KG_WRIST;
    slot0Wrist.kS = ArmConstants.KS_WRIST;
    slot0Wrist.kV = ArmConstants.KV_WRIST;
    slot0Wrist.kA = ArmConstants.KA_WRIST;
    slot0Wrist.kP = ArmConstants.KP_WRIST;
    slot0Wrist.kI = ArmConstants.KI_WRIST;
    slot0Wrist.kD = ArmConstants.KD_WRIST;

    // set Motion Magic settings - Shoulder
    var motionMagicConfigsShoulder = shoulderLeaderConfig.MotionMagic;
    motionMagicConfigsShoulder.MotionMagicCruiseVelocity = ArmConstants.SHOULDER_MAX_VELOCITY_RPS;
    motionMagicConfigsShoulder.MotionMagicExpo_kV = ArmConstants.KV_SHOULDER;
    motionMagicConfigsShoulder.MotionMagicExpo_kA = ArmConstants.KA_SHOULDER;
    motionMagicConfigsShoulder.MotionMagicAcceleration = ArmConstants.SHOULDER_MAX_ACCELERATION_RPS2;
    motionMagicConfigsShoulder.MotionMagicJerk = ArmConstants.SHOULDER_MAX_JERK_RPS3;

    // set Motion Magic settings - Elbow
    var motionMagicConfigsElbow = elbowConfiguration.MotionMagic;
    motionMagicConfigsElbow.MotionMagicCruiseVelocity = ArmConstants.ELBOW_MAX_VELOCITY_RPS;
    motionMagicConfigsElbow.MotionMagicExpo_kV = ArmConstants.KV_ELBOW;
    motionMagicConfigsElbow.MotionMagicExpo_kA = ArmConstants.KA_ELBOW;
    motionMagicConfigsElbow.MotionMagicAcceleration = ArmConstants.ELBOW_MAX_ACCELERATION_RPS2;
    motionMagicConfigsElbow.MotionMagicJerk = ArmConstants.ELBOW_MAX_JERK_RPS3;

    // set Motion Magic settings - Wrist
    var motionMagicConfigsWrist = wristConfiguration.MotionMagic;
    motionMagicConfigsWrist.MotionMagicCruiseVelocity = ArmConstants.WRIST_MAX_VELOCITY_RPS;
    motionMagicConfigsWrist.MotionMagicExpo_kV = ArmConstants.KV_WRIST;
    motionMagicConfigsWrist.MotionMagicExpo_kA = ArmConstants.KA_WRIST;
    motionMagicConfigsWrist.MotionMagicAcceleration = ArmConstants.WRIST_MAX_ACCELERATION_RPS2;
    motionMagicConfigsWrist.MotionMagicJerk = ArmConstants.WRIST_MAX_JERK_RPS3;


    // Apply configurations to the motors
    shoulderLeaderTalon.getConfigurator().apply(shoulderLeaderConfig);
    elbowLeaderTalon.getConfigurator().apply(elbowConfiguration);
    wristTalon.getConfigurator().apply(wristConfiguration);

    // Set follower control mode
    shoulderFollowerTalon.setControl(new Follower(shoulderLeaderTalon.getDeviceID(), false));
    elbowFollowerTalon.setControl(new Follower(elbowLeaderTalon.getDeviceID(), false));

    switch (ArmConstants.activeEncoders) {
      case RELATIVE:
        // Arm MUST be in correct position when deploying code or booting robot

        shoulderLeaderTalon.setPosition(ArmConstants.SHOULDER_STARTING_OFFSET.getRotations());
        elbowLeaderTalon.setPosition(ArmConstants.ELBOW_STARTING_OFFSET.getRotations());
        wristTalon.setPosition(ArmConstants.WRIST_STARTING_OFFSET.getRotations());
        break;
      case ABSOLUTE:
        break;

    }

    // Configure inputs
    shoulderPosition = shoulderLeaderTalon.getPosition();
    shoulderAppliedVolts = shoulderLeaderTalon.getMotorVoltage();
    shoulderCurrent = shoulderLeaderTalon.getSupplyCurrent();

    elbowPosition = elbowLeaderTalon.getPosition();
    elbowAppliedVolts = elbowLeaderTalon.getMotorVoltage();
    elbowCurrent = elbowLeaderTalon.getSupplyCurrent();

    wristPosition = wristTalon.getPosition();
    wristAppliedVolts = wristTalon.getMotorVoltage();
    wristCurrent = wristTalon.getSupplyCurrent();
  }

  @Override
  /**
   * Updates the inputs cerated in ArmIO to be real values from the motors.
   */
  public void updateInputs(ArmIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        shoulderPosition, shoulderAppliedVolts,
        shoulderCurrent, elbowPosition, elbowAppliedVolts,
        elbowCurrent);

    BaseStatusSignal.refreshAll(wristPosition, wristAppliedVolts, wristCurrent);

    inputs.shoulderCurrentAmps = new double[] { shoulderCurrent.getValueAsDouble() };
    inputs.shoulderPosition = Rotation2d.fromRotations(shoulderPosition.getValueAsDouble());

    inputs.elbowCurrentAmps = new double[] { elbowCurrent.getValueAsDouble() };
    inputs.elbowPosition = Rotation2d.fromRotations(elbowPosition.getValueAsDouble());

    inputs.wristCurrentAmps = new double[] { wristCurrent.getValueAsDouble() };
    inputs.wristPosition = Rotation2d.fromDegrees(Rotation2d.fromDegrees(90).minus(inputs.elbowPosition).getDegrees() + Rotation2d.fromRotations(wristPosition.getValueAsDouble()).getDegrees());
  }

  /**
   * Sets the shoulder motor position using MotionMagic
   * 
   * @param position A Rotation2d representing the position
   */
  @Override
  public void setShoulderPosition(Rotation2d position) {
    // create a Motion Magic request, voltage output
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    shoulderLeaderTalon.setControl(m_request.withPosition(position.getRotations()));
  }

  /**
   * Sets the elbow motor position using MotionMagic
   * 
   * @param position A Rotation2d representing the position
   */
  @Override
  public void setElbowPosition(Rotation2d position) {
    // create a Motion Magic request, voltage output
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    // set target position to 100 rotations
    elbowLeaderTalon.setControl(m_request.withPosition(position.getRotations()));
  }

  /**
   * Sets the wrist motor position using MotionMagic
   * 
   * @param position A Rotation2d representing the position
   */
  @Override
  public void setWristPosition(Rotation2d position) {
    // create a Motion Magic request, voltage output
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    wristTalon.setControl(m_request.withPosition(position.getRotations()));
  }
}
