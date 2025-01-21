// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ClimberConstants {
    
    //TODO: get real ports
    public static final int CLIMBER_MOTOR_PORT = 0;
    public static final int ARM_MOTOR_PORT = 0;

    public static final Rotation2d ARM_INIT_POSITION = Rotation2d.fromDegrees(0);
    public static final Rotation2d ARM_CLIMB_POSITION = Rotation2d.fromDegrees(0);

    public static final double ARM_FEED_FORWARD = 0.0;

}