package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class JoystickUtils {

  /**
   * squares the joystick input and compensates for the offset caused by the
   * deadband
   *
   * @param input The input from the joystick
   * @return The corrected joystick values
   */
  public static double curveInput(double input, double deadband) {

    // returns zero if input is less than deadband
    if (MathUtil.applyDeadband(input, deadband) == 0) return 0;

    double correctedValue = input;

    // does funky math to force linear output between deadband and 1
    correctedValue = (correctedValue - (deadband * Math.signum(correctedValue))) / (1 - deadband);

    // raises input to a specified power for a smoother feel
    correctedValue = Math.copySign(Math.pow(Math.abs(correctedValue), 2), input);

    return Math.min(correctedValue, 1.0);
  }
}