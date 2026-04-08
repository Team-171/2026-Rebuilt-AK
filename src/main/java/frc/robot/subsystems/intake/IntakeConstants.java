package frc.robot.subsystems.intake;

public final class IntakeConstants {

  public static final int intakeLeftWheelId = 32;
  public static final int intakeRightWheelId = 35;
  public static final int intakeLiftId = 34;

  public static final int stallCurrentLimit = 35;
  public static final int freeCurrentLimit = 40;

  public static final double kP = 1.8;
  public static final double kI = 0;
  public static final double kD = 0.01;

  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double cruiseVelocity = 5676;
  public static final double maxAcceleration = 190000;
  public static final double allowedError = 0.01;

  public static final double conversionFactor = (1.0 / 20);

  /** forward is down. exept for the limit setting. then its reverse */
  public static final double forwardLimit = 0.04;
  /** reverse is up. exept for the limit setting. that is forward */
  public static final double reverseLimit = 0.463;

  public static final double intakeSpeed = 0.50;
  public static double intakeSetpoint = 0;
}
