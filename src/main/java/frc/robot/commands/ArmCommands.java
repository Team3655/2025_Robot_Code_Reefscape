package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPose;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ArmCommands {
  public static Command updateSetpoint(ArmSubsystem arm, ArmPose setpoint){
    return Commands.runOnce(() -> arm.updateSetpoint(setpoint), arm);
  }

  public static Command toStart(ArmSubsystem arm, IntakeSubsystem intake) {
    
    if(arm.getSetPoint().name().equals(ArmConstants.ArmPoseNames.REAR_L4_REEF)){
      return Commands.sequence(
        IntakeCommands.stopIntake(intake),
        updateSetpoint(arm, ArmStates.REAR_L4_REEF_WRIST_FLIP),
        new WaitCommand(1 / (ArmConstants.WRIST_MAX_VELOCITY_RPS)),
        updateSetpoint(arm, ArmStates.REAR_L4_REEF_TRANSITION),
        new WaitCommand(0.2 / (ArmConstants.SHOULDER_MAX_VELOCITY_RPS)),
        updateSetpoint(arm, ArmStates.START));
    } else {
      return Commands.parallel(
        updateSetpoint(arm, ArmConstants.ArmStates.START), 
        IntakeCommands.stopIntake(intake));
    }
  }

  // Commands.sequence(
  //   IntakeCommands.stopIntake(intake),
  //   ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF_WRIST_FLIP),
  //   new WaitCommand(1 / (ArmConstants.WRIST_MAX_VELOCITY_RPS)),
  //   ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF_TRANSITION),
  //   new WaitCommand(0.2 / (ArmConstants.SHOULDER_MAX_VELOCITY_RPS)),
  //   ArmCommands.updateSetpoint(arm, ArmStates.START)));
}
