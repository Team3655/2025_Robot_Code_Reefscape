package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPose;

public class ArmCommands {
  public static Command updateSetpoint(ArmSubsystem arm, ArmPose setpoint){
    return Commands.runOnce(() -> arm.updateSetpoint(setpoint), arm);
  }

}
