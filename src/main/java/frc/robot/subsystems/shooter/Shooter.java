package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class Shooter extends SubsystemBase {

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
  }

  public void shooterVelocity(double vel) {
    motorPID.setSetpoint(vel, ControlType.kMAXMotionVelocityControl);
  }

  public void shoot() {
    // motorPID.setSetpoint(ShooterConstants.shootSpeed, ControlType.kMAXMotionVelocityControl);
    // TODO write wrapper for auto aiming
    shooterMotor.set(ShooterConstants.shootSpeed);
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {}
}
