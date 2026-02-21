package frc.robot.subsystems.intake;

public final class IntakeConstants {

  public static final int intakeWheelsId = 32;
  public static final int intakeLiftId = 31;

  public static final int stallCurrentLimit = 25;
  public static final int freeCurrentLimit = 40;

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double cruiseVelocity = 5676;
  public static final double maxAcceleration = 190000;
  public static final double allowedError = 1;

  public static final double conversionFactor = (1.0 / 20);

  public static final double forwardLimit = 90; // TODO
  public static final double reverseLimit = 0; // TODO

  public static final double intakeSpeed = 1;
}
