package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX shoulder;
    private final TalonFX elbow;
    private final TalonFX wrist;

    private final StatusSignal<Angle> shoulderPosition;
    private final StatusSignal<AngularVelocity> shoulderVelocity;
    private final StatusSignal<Voltage> shoulderAppliedVolts;
    private final StatusSignal<Current> shoulderCurrent;

    private final StatusSignal<Angle> elbowPosition;
    private final StatusSignal<AngularVelocity> elbowVelocity;
    private final StatusSignal<Voltage> elbowAppliedVolts;
    private final StatusSignal<Current> elbowCurrent;

    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<AngularVelocity> wristVelocity;
    private final StatusSignal<Voltage> wristAppliedVolts;
    private final StatusSignal<Current> wristCurrent;

    private final TalonFXConfiguration shoulderConfiguration;
    private final TalonFXConfiguration elbowConfiguration;
    private final TalonFXConfiguration wristConfiguration;

    public ArmIOTalonFX() {
        shoulder = new TalonFX(0, ArmConstants.CANBUS_NAME); // TODO: get real id
        elbow = new TalonFX(1, ArmConstants.CANBUS_NAME);
        wrist = new TalonFX(2, ArmConstants.CANBUS_NAME);

        shoulderConfiguration = new TalonFXConfiguration();
        elbowConfiguration = new TalonFXConfiguration();
        wristConfiguration = new TalonFXConfiguration();

        switch (ArmConstants.activeEncoders){
            case ABSOLUTE:

                CANcoder shoulderEncoder = new CANcoder(3, ArmConstants.CANBUS_NAME);
                CANcoder elbowEncoder = new CANcoder(4, ArmConstants.CANBUS_NAME);
                CANcoder wristEncoder = new CANcoder(5, ArmConstants.CANBUS_NAME);

                CANcoderConfiguration shoulderEncoderConfig = new CANcoderConfiguration();
                CANcoderConfiguration elbowEncoderConfig = new CANcoderConfiguration();
                CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();

                shoulderEncoderConfig.MagnetSensor.MagnetOffset = 
                    ArmConstants.SHOULDER_ENCODER_OFFSET.getRadians();
                elbowEncoderConfig.MagnetSensor.MagnetOffset =
                    ArmConstants.ELBOW_ENCODER_OFFSET.getRadians();
                wristEncoderConfig.MagnetSensor.MagnetOffset =
                    ArmConstants.WRIST_ENCODER_OFFSET.getRadians();

                shoulderEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
                elbowEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
                // TODO: Need to see constructed wrist to identify
                wristEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

                shoulderEncoder.getConfigurator().apply(shoulderEncoderConfig);
                elbowEncoder.getConfigurator().apply(elbowEncoderConfig);
                wristEncoder.getConfigurator().apply(wristEncoderConfig);

                shoulderConfiguration.Feedback.FeedbackRemoteSensorID = ArmConstants.SHOULDER_CANCODER_ID;
                elbowConfiguration.Feedback.FeedbackRemoteSensorID = ArmConstants.ELBOW_CANCODER_ID;
                wristConfiguration.Feedback.FeedbackRemoteSensorID = ArmConstants.WRIST_CANCODER_ID;

                shoulderConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                elbowConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                wristConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

                shoulderConfiguration.Feedback.RotorToSensorRatio = ArmConstants.SHOULDER_REDUCTION;
                elbowConfiguration.Feedback.RotorToSensorRatio = ArmConstants.ELBOW_REDUCTION;
                wristConfiguration.Feedback.RotorToSensorRatio = ArmConstants.WRIST_REDUCTION;

                shoulderConfiguration.Feedback.SensorToMechanismRatio = 1;
                elbowConfiguration.Feedback.SensorToMechanismRatio = 1;
                wristConfiguration.Feedback.SensorToMechanismRatio = 1;

                break;

            case RELATIVE:

                shoulderConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                elbowConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                wristConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

                shoulderConfiguration.Feedback.RotorToSensorRatio = 1;
                elbowConfiguration.Feedback.RotorToSensorRatio = 1;
                wristConfiguration.Feedback.RotorToSensorRatio = 1;

                shoulderConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.SHOULDER_REDUCTION;
                elbowConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.ELBOW_REDUCTION;
                wristConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.WRIST_REDUCTION;

                break;

            default:

                shoulderConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                elbowConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                wristConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

                shoulderConfiguration.Feedback.RotorToSensorRatio = 1;
                elbowConfiguration.Feedback.RotorToSensorRatio = 1;
                wristConfiguration.Feedback.RotorToSensorRatio = 1;

                shoulderConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.SHOULDER_REDUCTION;
                elbowConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.ELBOW_REDUCTION;
                wristConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.WRIST_REDUCTION;
                
                break;

        }

        var slot0Shoulder = new Slot0Configs();
        var slot0Elbow = new Slot0Configs();
        var slot0Wrist = new Slot0Configs();

        slot0Shoulder.kP = ArmConstants.KP_SHOULDER;
        slot0Shoulder.kD = ArmConstants.KD_SHOULDER;

        slot0Elbow.kP = ArmConstants.KP_ELBOW;
        slot0Elbow.kD = ArmConstants.KD_ELBOW;

        slot0Wrist.kP = ArmConstants.KP_WRIST;
        slot0Wrist.kD = ArmConstants.KD_WRIST;

        shoulder.getConfigurator().apply(shoulderConfiguration);
        elbow.getConfigurator().apply(elbowConfiguration);
        wrist.getConfigurator().apply(wristConfiguration);

        shoulderPosition = shoulder.getPosition();
        shoulderVelocity = shoulder.getVelocity();
        shoulderAppliedVolts = shoulder.getMotorVoltage();
        shoulderCurrent = shoulder.getSupplyCurrent();

        elbowPosition = elbow.getPosition();
        elbowAppliedVolts = elbow.getMotorVoltage();
        elbowVelocity = elbow.getVelocity();
        elbowCurrent = elbow.getSupplyCurrent();

        wristPosition = wrist.getPosition();
        wristVelocity = wrist.getVelocity();
        wristAppliedVolts = wrist.getMotorVoltage();
        wristCurrent = wrist.getSupplyCurrent();

    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        BaseStatusSignal.refreshAll(shoulderPosition, shoulderVelocity, shoulderAppliedVolts,
                shoulderCurrent, elbowPosition, elbowVelocity, elbowAppliedVolts, elbowCurrent,
                wristPosition, wristVelocity, wristAppliedVolts, wristCurrent);

        inputs.shoulderAppliedVolts = shoulderAppliedVolts.getValueAsDouble();
        inputs.shoulderCurrentAmps = new double[] { shoulderCurrent.getValueAsDouble() };
        inputs.shoulderPosition = Rotation2d.fromRotations(shoulderPosition.getValueAsDouble());
        inputs.shoulderVelocityRadPerSec = shoulderVelocity.getValueAsDouble();

        inputs.elbowAppliedVolts = elbowAppliedVolts.getValueAsDouble();
        inputs.elbowCurrentAmps = new double[] { elbowCurrent.getValueAsDouble() };
        inputs.elbowPosition = Rotation2d.fromRotations(elbowPosition.getValueAsDouble());
        inputs.elbowVelocityRadPerSec = elbowVelocity.getValueAsDouble();

        inputs.wristAppliedVolts = wristAppliedVolts.getValueAsDouble();
        inputs.wristCurrentAmps = new double[] { wristCurrent.getValueAsDouble() };
        inputs.wristPosition = Rotation2d.fromRotations(wristPosition.getValueAsDouble());
        inputs.wristVelocityRadPerSec = wristVelocity.getValueAsDouble();
    }

    @Override
    public void setShoulderPositionWithFeedForward(Rotation2d position, double feedForward) {
        shoulder.setControl(new PositionVoltage(position.getRotations()).withFeedForward(feedForward));
    }

    @Override
    public void setElbowPositionWithFeedForward(Rotation2d position, double feedForward) {
        elbow.setControl(new PositionVoltage(position.getRotations()).withFeedForward(feedForward));
    }

    @Override
    public void setWristPositionWithFeedForward(Rotation2d position, double feedForward) {
        wrist.setControl(new PositionVoltage(position.getRotations()).withFeedForward(feedForward));
    }

    @Override
    public void setShoulderVoltage(double volts) {
        shoulder.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElbowVoltage(double volts) {
        elbow.setControl(new VoltageOut(volts));
    }

    @Override
    public void setWristVoltage(double volts) {
        wrist.setControl(new VoltageOut(volts));
    }

}
