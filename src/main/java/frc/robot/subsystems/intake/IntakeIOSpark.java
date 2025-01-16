// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class IntakeIOSpark implements IntakeIO {
  private final SparkMax vacuumMotor;

  private final SparkMaxConfig config;

  private final Solenoid solenoid1;
  private final Solenoid solenoid2;
  private final Solenoid solenoid3;
  private final Solenoid solenoid4;
  private final Solenoid solenoidMain0;

  private final DigitalInput airSwitch1;
  private final DigitalInput airSwitch2;
  private final DigitalInput airSwitch3;
  private final DigitalInput airSwitch4;

  private final PneumaticsModuleType pneumaticsModule = PneumaticsModuleType.REVPH;

  public IntakeIOSpark() {
    vacuumMotor = new SparkMax(30, MotorType.kBrushless);

    config = new SparkMaxConfig();

    solenoid1 = new Solenoid(pneumaticsModule, 1);
    solenoid2 = new Solenoid(pneumaticsModule, 2);
    solenoid3 = new Solenoid(pneumaticsModule, 3);
    solenoid4 = new Solenoid(pneumaticsModule, 4);

    solenoidMain0 = new Solenoid(pneumaticsModule, 0);

    airSwitch1 = new DigitalInput(1);
    airSwitch2 = new DigitalInput(2);
    airSwitch3 = new DigitalInput(3);
    airSwitch4 = new DigitalInput(4);

    config.smartCurrentLimit(30);
    config.idleMode(IdleMode.kCoast);

    vacuumMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.airSwitch1Status = airSwitch1.get();
    inputs.airSwitch2Status = airSwitch2.get();
    inputs.airSwitch3Status = airSwitch3.get();
    inputs.airSwitch4Status = airSwitch4.get();
    inputs.vacuumVoltage = vacuumMotor.getBusVoltage();
  }

  @Override
  public boolean getAirSwitchState(int switchID) {
    switch (switchID) {
      case 1:
        return airSwitch1.get();
      case 2:
        return airSwitch2.get();
      case 3:
        return airSwitch3.get();
      case 4:
        return airSwitch4.get();
      default:
        return false;
    }
  }

  @Override
  public void setSolenoid(int solenoidID, boolean state) {
    switch (solenoidID) {
      case 1:
        solenoid1.set(state);
      case 2:
        solenoid2.set(state);
      case 3:
        solenoid3.set(state);
      case 4:
        solenoid4.set(state);
      default:
        break;
    }
  }

  @Override
  public void setVacuumVoltage(double voltage) {
      vacuumMotor.setVoltage(voltage);
  }

}
