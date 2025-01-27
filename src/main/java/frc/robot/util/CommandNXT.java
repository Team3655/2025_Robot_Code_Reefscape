package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandNXT extends CommandGenericHID {

  private static final int FIRE_BUTTON_STAGE1 = 1;
  private static final int FIRE_BUTTON_STAGE2 = 2;
  private static final int A2 = 3;
  private static final int B1 = 4;
  private static final int D1 = 5;
  private static final int A3_UP = 6;
  private static final int A3_RIGHT = 7;
  private static final int A3_DOWN = 8;
  private static final int A3_LEFT = 9;
  private static final int A3_IN = 10;
  private static final int A4_UP = 11;
  private static final int A4_RIGHT = 12;
  private static final int A4_DOWN = 13;
  private static final int A4_LEFT = 14;
  private static final int A4_IN = 15;
  private static final int C1_UP = 16;
  private static final int C1_RIGHT = 17;
  private static final int C1_DOWN = 18;
  private static final int C1_LEFT = 19;
  private static final int C1_IN = 20;
  private static final int FIRE_PADDLE_UP = 21;
  private static final int FIRE_PADDLE_DOWN = 22;
  private static final int EN1_UP = 23;
  private static final int EN1_DOWN = 24;
  private static final int SW1_UP = 25;
  private static final int SW1_DOWN = 26;
  private static final int SW2_ID = 2;
  private static final int Stick_X = 0;
  private static final int Stick_Y = 1;
  private static final int A1_X = 3;
  private static final int A1_Y = 4;
  private static final int Stick_Z = 5;

  private final GenericHID hid;

  public CommandNXT(int port) {
    super(port);
    hid = new GenericHID(port);
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the first stage is depressed but not
   *         when the second is
   */
  public Trigger fireStage1() {
    return new Trigger(
        () -> hid.getRawButton(FIRE_BUTTON_STAGE1) && !hid.getRawButton(FIRE_BUTTON_STAGE2));
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the second stage is depressed
   */
  public Trigger fireStage2() {
    return new Trigger(() -> hid.getRawButton(FIRE_BUTTON_STAGE2));
  }

  // second paddle pushed up
  public Trigger firePaddleUp() {
    return new Trigger(() -> hid.getRawButton(FIRE_PADDLE_UP));
  }

  // second paddle pushed down
  public Trigger firePaddleDown() {
    return new Trigger(() -> hid.getRawButton(FIRE_PADDLE_DOWN));
  }

  // red button
  public Trigger A2() {
    return new Trigger(() -> hid.getRawButton(A2));
  }

  // button on top and back of controller
  public Trigger B1() {
    return new Trigger(() -> hid.getRawButton(B1));
  }

  // button on the bottom and back of controller
  public Trigger D1() {
    return new Trigger(() -> hid.getRawButton(D1));

  }

  // middle joystick up
  public Trigger A3_UP() {
    return new Trigger(() -> hid.getRawButton(A3_UP));
  }

  // middle joystick right
  public Trigger A3_RIGHT() {
    return new Trigger(() -> hid.getRawButton(A3_RIGHT));
  }

  // middle joystick down
  public Trigger A3_DOWN() {
    return new Trigger(() -> hid.getRawButton(A3_DOWN));
  }

  // middle joystick left
  public Trigger A3_LEFT() {
    return new Trigger(() -> hid.getRawButton(A3_LEFT));
  }

  // middle joystick in
  public Trigger A3_IN() {
    return new Trigger(() -> hid.getRawButton(A3_IN));
  }

  // top right joystick up
  public Trigger A4_UP() {
    return new Trigger(() -> hid.getRawButton(A4_UP));
  }

  // top right joystick in
  public Trigger A4_RIGHT() {
    return new Trigger(() -> hid.getRawButton(A4_RIGHT));
  }

  // top right joystick down
  public Trigger A4_DOWN() {
    return new Trigger(() -> hid.getRawButton(A4_DOWN));
  }

  // top right joystick left
  public Trigger A4_LEFT() {
    return new Trigger(() -> hid.getRawButton(A4_LEFT));
  }

  // top right joystick in
  public Trigger A4_IN() {
    return new Trigger(() -> hid.getRawButton(A4_IN));
  }

  // left gray stick up
  public Trigger C1_UP() {
    return new Trigger(() -> hid.getRawButton(C1_UP));
  }

  // left gray stick right
  public Trigger C1_RIGHT() {
    return new Trigger(() -> hid.getRawButton(C1_RIGHT));
  }

  // left gray stick down
  public Trigger C1_DOWN() {
    return new Trigger(() -> hid.getRawButton(C1_DOWN));
  }

  // left gray stick left
  public Trigger C1_LEFT() {
    return new Trigger(() -> hid.getRawButton(C1_LEFT));
  }

  // left gray stick in
  public Trigger C1_IN() {
    return new Trigger(() -> hid.getRawButton(C1_IN));
  }

  // bottom right wheel up
  public Trigger EN1_UP() {
    return new Trigger(() -> hid.getRawButton(EN1_UP));
  }

  // bottom right wheel down
  public Trigger EN1_DOWN() {
    return new Trigger(() -> hid.getRawButton(EN1_DOWN));
  }

  // bottom left wheel up
  public Trigger SW1_UP() {
    return new Trigger(() -> hid.getRawButton(SW1_UP));
  }

  // bottom left wheel down
  public Trigger SW1_DOWN() {
    return new Trigger(() -> hid.getRawButton(SW1_DOWN));
  }

  // bottom middle wheel
  public double SW2() {
    return hid.getRawAxis(SW2_ID);
  }

  // main stick forward and backward
  public double StickYAxis() {
    return hid.getRawAxis(Stick_Y);
  }

  // main stick left and right
  public double StickXAxis() {
    return hid.getRawAxis(Stick_X);
  }

  // main stick rotation
  public double StickZAxis() {
    return hid.getRawAxis(Stick_Z);
  }

  // top left stick left and right
  public double A1XAxis() {
    return hid.getRawAxis(A1_X);
  }

  // top left stick up and down
  public double A1YAxis() {
    return hid.getRawAxis(A1_Y);
  }
}