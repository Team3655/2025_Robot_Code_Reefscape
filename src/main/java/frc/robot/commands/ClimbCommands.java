// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

/** Add your docs here. */
public class ClimbCommands {

    public static Command runClimber(ClimberSubsystem climber, double volts){
        return Commands.runOnce(() -> climber.updateClimberVoltate(volts), climber);
    }

    public static Command stopClimber(ClimberSubsystem climber){
        return Commands.runOnce(() -> climber.updateClimberVoltate(0), climber);
    }

    public static Command setArmClimbPosition(ClimberSubsystem climber){
        return Commands.runOnce(() -> climber.updateArmPosition(ClimberConstants.ARM_CLIMB_POSITION), climber);
    }

    public static Command setArmInitPosition(ClimberSubsystem climber){
        return Commands.runOnce(() -> climber.updateArmPosition(ClimberConstants.ARM_INIT_POSITION), climber);
    }
}
