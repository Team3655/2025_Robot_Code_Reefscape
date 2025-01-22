package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX climberMotor;
  private final TalonFX armMotor;

  private final StatusSignal<AngularVelocity> climberVelocity;
  private final StatusSignal<Voltage> climberAppliedVolts;
  private final StatusSignal<Current> climberCurrent;

  private final StatusSignal<Angle> armPosition;
  private final StatusSignal<AngularVelocity> armVelocity;
  private final StatusSignal<Voltage> armAppliedVolts;
  private final StatusSignal<Current> armCurrent;

  public ClimberIOTalonFX() {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_PORT);
    armMotor = new TalonFX(ClimberConstants.ARM_MOTOR_PORT);

    climberVelocity = climberMotor.getVelocity();
    climberAppliedVolts = climberMotor.getMotorVoltage();
    climberCurrent = climberMotor.getSupplyCurrent();

    armPosition = armMotor.getPosition();
    armVelocity = armMotor.getVelocity();
    armAppliedVolts = armMotor.getMotorVoltage();
    armCurrent = armMotor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberVelocityRadPerSec = climberVelocity.getValueAsDouble();
    inputs.climberAppliedVolts = climberAppliedVolts.getValueAsDouble();
    inputs.climberCurrentAmps = new double[] { climberCurrent.getValueAsDouble() };

    inputs.armPosition = Rotation2d.fromRotations(armPosition.getValueAsDouble());
    inputs.armVelocityRadPerSec = armVelocity.getValueAsDouble();
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armCurrentAmps = new double[] { armCurrent.getValueAsDouble() };
  }

  @Override
  public void setClimberVoltage(double volts) {
    climberMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setArmVoltage(double volts) {
    armMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setArmPosition(Rotation2d position, double feedForward) {
    armMotor.setControl(new PositionVoltage(position.getRotations()).withFeedForward(feedForward));
  }

}
