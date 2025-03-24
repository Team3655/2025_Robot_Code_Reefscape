// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmConstants.ArmPose;
import frc.robot.subsystems.arm.ArmSubsystem;

public class EnforcedArmSetpoint extends Command {
    
    private ArmSubsystem arm;
    private ArmPose pose;
    private double toleranceDegrees;

    public EnforcedArmSetpoint(ArmSubsystem arm, ArmPose pose, double toleranceDegrees) {
      this.arm = arm;
      this.pose = pose;
      this.toleranceDegrees = toleranceDegrees;
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      arm.updateSetpoint(pose);
    }

    @Override
    public boolean isFinished() {
      if(arm.isAtSetpoint(toleranceDegrees)) {
        return true;
      }
      return false;
    }
}
