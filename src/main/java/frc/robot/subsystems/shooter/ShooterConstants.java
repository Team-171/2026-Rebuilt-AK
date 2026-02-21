package frc.robot.subsystems.shooter;

public final class ShooterConstants {

  public static final int shooterCanId = 33;

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

  public static final double conversionFactor = (1.0 / 9); // TODO

  public static final double shootSpeed = 0.40;

  public static final double shooterkP = 5;
  public static final double shooterkI = 0;
  public static final double shooterkD = 0;
}
