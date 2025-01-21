// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


/** Add your docs here. */
public interface ClimberIO {

    @AutoLog
    public class ClimberIOInputs {

        public double climberVelocityRadPerSec = 0.0;
        public double climberAppliedVolts = 0.0;
        public double[] climberCurrentAmps = new double[] {};

        public Rotation2d armPosition = new Rotation2d();
        public double armVelocityRadPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double[] armCurrentAmps = new double[] {};
    }
    
    public default void updateInputs(ClimberIOInputs inputs) {}
    public default void setClimberVoltage(double volts) {}
    public default void setArmVoltage(double volts) {}
    public default void setArmPosition(Rotation2d position, double feedForward){}
} 
