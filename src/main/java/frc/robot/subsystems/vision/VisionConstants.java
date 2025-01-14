// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/** Add your docs here. */
public class VisionConstants {

  public static final double TRANSLATION_COEFFICIENT = 0.55;
  public static final double ROTATION_COEFFICIENT = 2.0;

  // the maximum distance a measurement will be accepted in meters
  public static final double SINGLE_TAG_MAXIMUM = 4.5;
  public static final double MULTI_TAG_MAXIMUM = 7.5;

  public enum VisionMode {
    APRILTAG_LOCALIZE,
    DETECT_GAMEPIECE,
    RETRO_REFLECTIVE
  }
}
