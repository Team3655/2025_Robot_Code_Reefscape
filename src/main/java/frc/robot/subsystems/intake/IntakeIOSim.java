// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.PneumaticsBaseSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {


    private int HUB_CAN_ID = 31;

    private DCMotorSim vacMotor = new DCMotorSim(null, null, null);

    private final SolenoidSim solenoidMain0 = 
        new SolenoidSim(
            PneumaticsBaseSim.getForType(HUB_CAN_ID, PneumaticsModuleType.REVPH), 
            0);

    private final SolenoidSim solenoid1 = 
        new SolenoidSim(
            PneumaticsBaseSim.getForType(HUB_CAN_ID, PneumaticsModuleType.REVPH), 
            1);

    private final SolenoidSim solenoid2 = 
        new SolenoidSim(
            PneumaticsBaseSim.getForType(HUB_CAN_ID, PneumaticsModuleType.REVPH), 
            2);

    private final SolenoidSim solenoid3 = 
        new SolenoidSim(
            PneumaticsBaseSim.getForType(HUB_CAN_ID, PneumaticsModuleType.REVPH), 
            3);

    private final SolenoidSim solenoid4 = 
        new SolenoidSim(
            PneumaticsBaseSim.getForType(HUB_CAN_ID, PneumaticsModuleType.REVPH), 
            4);

    private SolenoidSim[] solenoids = new SolenoidSim[5];

    public IntakeIOSim() {

        solenoids[0] = solenoidMain0;
        solenoids[1] = solenoid1;
        solenoids[2] = solenoid2;
        solenoids[3] = solenoid3;
        solenoids[4] = solenoid4;
    }

    @Override
    public void setSolenoid(int solenoidID, boolean state) {
        solenoids[solenoidID].setOutput(state);
    }
    
    @Override
    public void setVacuumVoltage(double voltage) {

    }
}
