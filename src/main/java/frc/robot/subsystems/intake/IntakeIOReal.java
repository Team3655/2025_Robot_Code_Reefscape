package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {

  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT, IntakeConstants.BUS);
  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private final CANrangeConfiguration canRangeConfig;

  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;
  private final StatusSignal<Temperature> intakeTemp;

  public IntakeIOReal() {
    intakeVelocity = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getSupplyCurrent();
    intakeTemp = intakeMotor.getDeviceTemp();

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ProximityParams.ProximityThreshold = IntakeConstants.CANRANGE_DETECTION_RANGE;
    
    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocityRadPerSec = intakeVelocity.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = new double[] { intakeCurrent.getValueAsDouble() };
    inputs.intakeTemp = intakeTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setControl(new VoltageOut(volts));
  }
}
