package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final InterpolatingDoubleTreeMap treemap = new InterpolatingDoubleTreeMap();

  private final SparkMax shooterMotor =
      new SparkMax(ShooterConstants.shooterCanId, MotorType.kBrushless);

  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController motorPID = shooterMotor.getClosedLoopController();

  public Shooter() {
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ShooterConstants.stallCurrentLimit, ShooterConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(true);
    motorConfig
        .closedLoop
        .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
        .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    motorConfig
        .closedLoop
        .maxMotion
        .cruiseVelocity(ShooterConstants.cruiseVelocity)
        .maxAcceleration(ShooterConstants.maxAcceleration)
        .allowedProfileError(ShooterConstants.allowedError);
    motorConfig
        .encoder
        .positionConversionFactor(ShooterConstants.conversionFactor)
        .velocityConversionFactor(ShooterConstants.conversionFactor);

    SparkUtil.tryUntilOk(
        shooterMotor,
        5,
        () ->
            shooterMotor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    treemap.put(-1.0, 0.0);
    treemap.put(0.0, 0.5);
    treemap.put(1.0, 1.0);
  }

  public void shooterVelocity(double vel) {
    motorPID.setSetpoint(vel, ControlType.kMAXMotionVelocityControl);
  }

  public void shoot(double speed) {
    // motorPID.setSetpoint(ShooterConstants.shootSpeed, ControlType.kMAXMotionVelocityControl);
    // TODO write wrapper for auto aiming
    Logger.recordOutput("Shooter/ConstSpeed", mapped(speed));
    shooterMotor.set(mapped(speed));
  }

  public static double mapped(double value) {
    return MathUtil.clamp((value + 1) / 2.0, 0.0, 1.0);
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/Velocity", shooterMotor.getEncoder().getVelocity());
  }
}
