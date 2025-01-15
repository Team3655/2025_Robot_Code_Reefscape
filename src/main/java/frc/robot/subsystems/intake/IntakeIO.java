// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs {
        public boolean airSwitch1Status = false;
        public boolean airSwitch2Status = false;
        public boolean airSwitch3Status = false;
        public boolean airSwitch4Status = false;

        public double vacuumVoltage = 0.0;
    }

    public default void setSolenoid(int solenoidID, boolean state) {}
    public default void setVacuumVoltage(double voltage) {}
    public default boolean getSolenoidState() {
        return false;
    }
    public default boolean getAirSwitchState(int switchID) {
        return false;
    }
    public default void updateInputs() {}
}
