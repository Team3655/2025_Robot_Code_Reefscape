package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX vacMotor = new TalonFX(IntakeConstants.VAC_MOTOR_PORT, IntakeConstants.BUS);
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT, IntakeConstants.BUS);
    private final DigitalInput frontSensor = new DigitalInput(0);
    private final DigitalInput backSensor = new DigitalInput(0);

    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> intakeAppliedVolts;
    private final StatusSignal<Current> intakeCurrent;

    private final boolean frontSensorState = frontSensor.get();
    private final boolean backSensorState = backSensor.get();

    private final StatusSignal<Voltage> vacuumAppliedVolts;

    public IntakeIOTalonFX() {
        intakeVelocity = intakeMotor.getVelocity();
        intakeAppliedVolts = intakeMotor.getMotorVoltage();
        intakeCurrent = intakeMotor.getSupplyCurrent();

        vacuumAppliedVolts = vacMotor.getMotorVoltage();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocityRadPerSec = intakeVelocity.getValueAsDouble();
        inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
        inputs.intakeCurrentAmps = new double[] { intakeCurrent.getValueAsDouble() };

        inputs.vacuumAppliedVolts = vacuumAppliedVolts.getValueAsDouble();

        inputs.frontSensorState = frontSensorState;
        inputs.backSensorState = backSensorState;  
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setVacuumVoltage(double voltage) {
        vacMotor.setControl(new VoltageOut(voltage));
    }
}
