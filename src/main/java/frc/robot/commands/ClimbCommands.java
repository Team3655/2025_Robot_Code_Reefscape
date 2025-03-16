// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberSubsystem;

/** Add your docs here. */
public class ClimbCommands {

    private static final double maxClimberVoltage = 6;

    public static Command runClimber(ClimberSubsystem climber, double volts){
        return Commands.runOnce(() -> climber.updateClimberVoltage(volts), climber);
    }

    public static Command stopClimber(ClimberSubsystem climber){
        return Commands.runOnce(() -> climber.updateClimberVoltage(0), climber);
    }

    public static Command driveClimber(ClimberSubsystem climber, DoubleSupplier positiveVolts, DoubleSupplier negativeVolts) {
        return Commands.run(() -> climber.driveClimber(maxClimberVoltage * positiveVolts.getAsDouble() - maxClimberVoltage * negativeVolts.getAsDouble()), climber);
    }

}
