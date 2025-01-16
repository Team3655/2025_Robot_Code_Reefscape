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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
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

  private final Solenoid[] solenoids = new Solenoid[4];

  private final DigitalInput airSwitch1;
  private final DigitalInput airSwitch2;
  private final DigitalInput airSwitch3;
  private final DigitalInput airSwitch4;

  private final DigitalInput[] airSwitches = new DigitalInput[4];

  private final PneumaticHub pneumaticHub;

  private final int HUB_CAN_ID = 31;

  public IntakeIOSpark() {
    vacuumMotor = new SparkMax(30, MotorType.kBrushless);

    config = new SparkMaxConfig();

    pneumaticHub = new PneumaticHub(HUB_CAN_ID);

    solenoidMain0 = pneumaticHub.makeSolenoid(0);
    solenoid1 = pneumaticHub.makeSolenoid(1);
    solenoid2 = pneumaticHub.makeSolenoid(2);
    solenoid3 = pneumaticHub.makeSolenoid(3);
    solenoid4 = pneumaticHub.makeSolenoid(4);

    solenoids[0] = solenoidMain0;
    solenoids[1] = solenoid1;
    solenoids[2] = solenoid2;
    solenoids[3] = solenoid3;
    solenoids[4] = solenoid4;

    airSwitch1 = new DigitalInput(1);
    airSwitch2 = new DigitalInput(2);
    airSwitch3 = new DigitalInput(3);
    airSwitch4 = new DigitalInput(4);

    airSwitches[1] = airSwitch1;
    airSwitches[2] = airSwitch2;
    airSwitches[3] = airSwitch3;
    airSwitches[4] = airSwitch4;


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
    return airSwitches[switchID].get();
  }

  @Override
  public void setSolenoid(int solenoidID, boolean state) {
    solenoids[solenoidID].set(state);
  }

  @Override
  public void setVacuumVoltage(double voltage) {
    vacuumMotor.setVoltage(voltage);
  }

}
