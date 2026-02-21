package frc.robot.subsystems.intake;

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

public class Intake extends SubsystemBase {

  private final SparkMax wheelMotor =
      new SparkMax(IntakeConstants.intakeWheelsId, MotorType.kBrushless);
  private final SparkMax liftMotor =
      new SparkMax(IntakeConstants.intakeLiftId, MotorType.kBrushless);

  private final SparkMaxConfig wheelMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig liftMotorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController liftPID = liftMotor.getClosedLoopController();

  private boolean isIntakeDeployed = false;

  public Intake() {
    wheelMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.stallCurrentLimit, IntakeConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(false);

    SparkUtil.tryUntilOk(
        wheelMotor,
        5,
        () ->
            wheelMotor.configure(
                wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    liftMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.stallCurrentLimit, IntakeConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(false);
    liftMotorConfig
        .closedLoop
        .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
        .outputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);
    liftMotorConfig
        .closedLoop
        .maxMotion
        .cruiseVelocity(IntakeConstants.cruiseVelocity)
        .maxAcceleration(IntakeConstants.maxAcceleration)
        .allowedProfileError(IntakeConstants.allowedError);
    liftMotorConfig
        .encoder
        .positionConversionFactor(IntakeConstants.conversionFactor)
        .velocityConversionFactor(IntakeConstants.conversionFactor);
    liftMotorConfig
        .softLimit
        .forwardSoftLimit(IntakeConstants.forwardLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(IntakeConstants.reverseLimit)
        .reverseSoftLimitEnabled(true);

    SparkUtil.tryUntilOk(
        liftMotor,
        5,
        () ->
            liftMotor.configure(
                liftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void toggleIntake() {
    if (isIntakeDeployed) {
      liftPID.setSetpoint(0, ControlType.kMAXMotionPositionControl); // TODO Figure out setpoints
      isIntakeDeployed = false;
    } else {
      liftPID.setSetpoint(90, ControlType.kMAXMotionPositionControl); // TODO Figure out setpoints
      isIntakeDeployed = true;
    }
  }

  public void intake() {
    wheelMotor.set(IntakeConstants.intakeSpeed);
  }

  public void reverseIntake() {
    wheelMotor.set(-IntakeConstants.intakeSpeed);
  }

  public void stopIntake() {
    wheelMotor.stopMotor();
  }

  public void stopAllIntakeMotors() {
    wheelMotor.stopMotor();
    liftMotor.stopMotor();
  }

  @Override
  public void periodic() {}
}
