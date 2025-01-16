package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX shoulder;
    private final TalonFX elbow;
    private final TalonFX wrist;

    // TODO: switch these to absolute encoders

    private final CANcoder shoulderEncoder;
    private final CANcoder elbowEncoder;
    private final CANcoder wristEncoder;

    private final Rotation2d shoulderEncoderOffset;
    private final Rotation2d elbowEncoderOffset;
    private final Rotation2d wristEncoderOffset;

    private final StatusSignal<Angle> shoulderAbsolutePosition;
    private final StatusSignal<Angle> shoulderPosition;
    private final StatusSignal<AngularVelocity> shoulderVelocity;
    private final StatusSignal<Voltage> shoulderAppliedVolts;
    private final StatusSignal<Current> shoulderCurrent;

    private final StatusSignal<Angle> elbowAbsolutePosition;
    private final StatusSignal<Angle> elbowPosition;
    private final StatusSignal<AngularVelocity> elbowVelocity;
    private final StatusSignal<Voltage> elbowAppliedVolts;
    private final StatusSignal<Current> elbowCurrent;

    private final StatusSignal<Angle> wristAbsolutePosition;
    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<AngularVelocity> wristVelocity;
    private final StatusSignal<Voltage> wristAppliedVolts;
    private final StatusSignal<Current> wristCurrent;

    public ArmIOTalonFX() {
        shoulder = new TalonFX(0, ArmConstants.CANBUS_NAME); // TODO: get real id and canbus name
        elbow = new TalonFX(1, ArmConstants.CANBUS_NAME);
        wrist = new TalonFX(2, ArmConstants.CANBUS_NAME);

        shoulderEncoder = new CANcoder(3, ArmConstants.CANBUS_NAME);
        elbowEncoder = new CANcoder(4, ArmConstants.CANBUS_NAME);
        wristEncoder = new CANcoder(5, ArmConstants.CANBUS_NAME);

        shoulderEncoderOffset = ArmConstants.SHOULDER_ENCODER_OFFSET;
        elbowEncoderOffset = ArmConstants.ELBOW_ENCODER_OFFSET;
        wristEncoderOffset = ArmConstants.WRIST_ENCODER_OFFSET;

        shoulderAbsolutePosition = shoulderEncoder.getAbsolutePosition();
        shoulderPosition = shoulder.getPosition();
        shoulderVelocity = shoulder.getVelocity();
        shoulderAppliedVolts = shoulder.getMotorVoltage();
        shoulderCurrent = shoulder.getSupplyCurrent();

        elbowAbsolutePosition = elbowEncoder.getAbsolutePosition();
        elbowPosition = elbow.getPosition();
        elbowAppliedVolts = elbow.getMotorVoltage();
        elbowVelocity = elbow.getVelocity();
        elbowCurrent = elbow.getSupplyCurrent();

        wristAbsolutePosition = wristEncoder.getAbsolutePosition();
        wristPosition = wrist.getPosition();
        wristVelocity = wrist.getVelocity();
        wristAppliedVolts = wrist.getMotorVoltage();
        wristCurrent = wrist.getSupplyCurrent();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(shoulderAbsolutePosition, shoulderPosition, shoulderVelocity, shoulderAppliedVolts,
                shoulderCurrent, elbowAbsolutePosition, elbowPosition, elbowVelocity, elbowAppliedVolts, elbowCurrent,
                wristAbsolutePosition, wristPosition, wristVelocity, wristAppliedVolts, wristCurrent);

        inputs.shoulderAbsolutePosition = Rotation2d.fromRotations(shoulderAbsolutePosition.getValueAsDouble())
                .minus(shoulderEncoderOffset);
        inputs.shoulderAppliedVolts = shoulderAppliedVolts.getValueAsDouble();
        inputs.shoulderCurrentAmps = new double[] { shoulderCurrent.getValueAsDouble() };
        inputs.shoulderPosition = Rotation2d.fromRotations(shoulderPosition.getValueAsDouble());
        inputs.shoulderVelocityRadPerSec = shoulderVelocity.getValueAsDouble();

        inputs.elbowAbsolutePosition = Rotation2d.fromRotations(elbowAbsolutePosition.getValueAsDouble())
                .minus(elbowEncoderOffset);
        inputs.elbowAppliedVolts = elbowAppliedVolts.getValueAsDouble();
        inputs.elbowCurrentAmps = new double[] { elbowCurrent.getValueAsDouble() };
        inputs.elbowPosition = Rotation2d.fromRotations(elbowPosition.getValueAsDouble());
        inputs.elbowVelocityRadPerSec = elbowVelocity.getValueAsDouble();

        inputs.wristAbsolutePosition = Rotation2d.fromRotations(wristAbsolutePosition.getValueAsDouble())
                .minus(wristEncoderOffset);
        inputs.wristAppliedVolts = wristAppliedVolts.getValueAsDouble();
        inputs.wristCurrentAmps = new double[] { wristCurrent.getValueAsDouble() };
        inputs.wristPosition = Rotation2d.fromRotations(wristPosition.getValueAsDouble());
        inputs.wristVelocityRadPerSec = wristVelocity.getValueAsDouble();
    }

    @Override
    public void setShoulderVoltage(double volts) {
        shoulder.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElbowVoltage(double volts){
        elbow.setControl(new VoltageOut(volts));
    }

    @Override
    public void setWristVoltage(double volts){
        wrist.setControl(new VoltageOut(volts));
    }

}
