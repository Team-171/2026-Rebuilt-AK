package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        .idleMode(IdleMode.kCoast)
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
        .forwardSoftLimitEnabled(true) // MARK: TRUE
        .reverseSoftLimit(IntakeConstants.reverseLimit)
        .reverseSoftLimitEnabled(true); // MARK: TRUE

    SparkUtil.tryUntilOk(
        liftMotor,
        5,
        () ->
            liftMotor.configure(
                liftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void toggleIntake() {
    if (isIntakeDeployed) {
      // liftPID.setSetpoint(0, ControlType.kMAXMotionPositionControl); // TODO Figure out setpoints
      IntakeConstants.intakeSetpoint = 0;
      isIntakeDeployed = false;
    } else {
      // liftPID.setSetpoint(1.8, ControlType.kMAXMotionPositionControl); // TODO Figure out
      // setpoints
      isIntakeDeployed = true;
      IntakeConstants.intakeSetpoint = 1.8;
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

  public Command intakeManual(double power) {
    return Commands.run(() -> liftMotor.set(power));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift Position", liftMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("Lift PID Setpoint Position",
    // liftPID.getMAXMotionSetpointPosition());
    // SmartDashboard.putNumber("Lift PID Setpoint", liftPID.getSetpoint());
    liftPID.setSetpoint(IntakeConstants.intakeSetpoint, ControlType.kMAXMotionPositionControl);
  }
}
