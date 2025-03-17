package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX climberMotor;
  private final TalonFXConfiguration climberConfiguration;

  private final StatusSignal<AngularVelocity> climberVelocity;
  private final StatusSignal<Voltage> climberAppliedVolts;
  private final StatusSignal<Current> climberCurrent;
  private final StatusSignal<Angle> climberPosition;

  public ClimberIOTalonFX() {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_PORT, "rio");

    climberConfiguration = new TalonFXConfiguration();
    climberConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    climberConfiguration.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;
    climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor.getConfigurator().apply(climberConfiguration);
    climberMotor.setPosition(0);

    climberVelocity = climberMotor.getVelocity();
    climberAppliedVolts = climberMotor.getMotorVoltage();
    climberCurrent = climberMotor.getSupplyCurrent();
    climberPosition = climberMotor.getPosition();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    BaseStatusSignal.refreshAll(climberVelocity, climberAppliedVolts, climberCurrent, climberPosition);

    inputs.climberVelocityRadPerSec = climberVelocity.getValueAsDouble();
    inputs.climberAppliedVolts = climberAppliedVolts.getValueAsDouble();
    inputs.climberCurrentAmps = new double[] { climberCurrent.getValueAsDouble() };
    inputs.climberPosition = Rotation2d.fromRotations(climberPosition.getValueAsDouble());
  }

  @Override
  public void setClimberVoltage(double volts) {
    climberMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void driveClimber(double volts) {
      climberMotor.setVoltage(volts);
  }
}
