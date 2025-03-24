package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants.ArmPose;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmCommands {
  public static Command updateSetpoint(ArmSubsystem arm, ArmPose pose){
    return Commands.runOnce(() -> arm.updateSetpoint(pose), arm);
  }

  public static SequentialCommandGroup transitionSetpoint(ArmSubsystem arm, ArmPose targetPose, ArmPose transitionPose) {
    return new SequentialCommandGroup(
      new EnforcedArmSetpoint(arm, transitionPose, 2),
      updateSetpoint(arm, targetPose)
    );
  } 
}
