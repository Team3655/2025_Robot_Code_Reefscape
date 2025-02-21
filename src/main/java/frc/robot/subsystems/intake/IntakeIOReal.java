package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {

  private final SparkMax vacMotor = new SparkMax(IntakeConstants.VAC_MOTOR_PORT, MotorType.kBrushless);

  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT, IntakeConstants.BUS);
  //private final CANrange canRange = new CANrange(IntakeConstants.CANRANGE_PORT, IntakeConstants.BUS);

  private final CANrangeConfiguration canRangeConfig;

  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;
  private final StatusSignal<Temperature> intakeTemp;

  private final double vacuumAppliedVolts;
  private final double vacuumTemp;

  public IntakeIOReal() {
    intakeVelocity = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getSupplyCurrent();
    intakeTemp = intakeMotor.getDeviceTemp();

    vacuumAppliedVolts = vacMotor.getAppliedOutput();
    vacuumTemp = vacMotor.getMotorTemperature();

    canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ProximityParams.ProximityThreshold = IntakeConstants.CANRANGE_DETECTION_RANGE;
    
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocityRadPerSec = intakeVelocity.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = new double[] { intakeCurrent.getValueAsDouble() };
    inputs.intakeTemp = intakeTemp.getValueAsDouble();

    inputs.vacuumAppliedVolts = vacuumAppliedVolts;
    inputs.vacuumTemp = vacuumTemp;

    //inputs.hasCoral = canRange.getIsDetected().getValue();
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVacuumVoltage(double voltage) {
    vacMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }
}
