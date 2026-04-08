package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.CURRENTCOLOR;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final SparkMax wheelMotorLeft =
      new SparkMax(IntakeConstants.intakeLeftWheelId, MotorType.kBrushless);
  private final SparkMax wheelMotorRight =
      new SparkMax(IntakeConstants.intakeRightWheelId, MotorType.kBrushless);
  private final SparkMax liftMotor =
      new SparkMax(IntakeConstants.intakeLiftId, MotorType.kBrushless);

  private final SparkMaxConfig leftWheelMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightWheelMotorConfig = new SparkMaxConfig();

  private final SparkMaxConfig liftMotorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController liftPID = liftMotor.getClosedLoopController();

  @SuppressWarnings("FieldMayBeFinal")
  private boolean isIntakeDeployed = false;

  public Intake() {
    leftWheelMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.stallCurrentLimit, IntakeConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(true);
    rightWheelMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.stallCurrentLimit, IntakeConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(false);

    SparkUtil.tryUntilOk(
        wheelMotorLeft,
        5,
        () ->
            wheelMotorLeft.configure(
                leftWheelMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        wheelMotorRight,
        5,
        () ->
            wheelMotorRight.configure(
                rightWheelMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    liftMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.stallCurrentLimit, IntakeConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(true);
    liftMotorConfig
        .closedLoop
        .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
        .outputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    liftMotorConfig
        .closedLoop
        .maxMotion
        .cruiseVelocity(IntakeConstants.cruiseVelocity)
        .maxAcceleration(IntakeConstants.maxAcceleration)
        .allowedProfileError(IntakeConstants.allowedError);
    /* liftMotorConfig
    .encoder
    .positionConversionFactor(IntakeConstants.conversionFactor)
    .velocityConversionFactor(IntakeConstants.conversionFactor); */
    liftMotorConfig
        .softLimit
        // the forward limit is the reverse limit. dont change
        .forwardSoftLimit(IntakeConstants.reverseLimit)
        .forwardSoftLimitEnabled(true) // MARK: TRUE
        // the reverse limit is the forward limit. dont change
        .reverseSoftLimit(IntakeConstants.forwardLimit)
        .reverseSoftLimitEnabled(true); // MARK: TRUE

    SparkUtil.tryUntilOk(
        liftMotor,
        5,
        () ->
            liftMotor.configure(
                liftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public Command intakeDeploy() {
    return Commands.run(
            () -> {
              liftPID.setSetpoint(
                  IntakeConstants.forwardLimit, ControlType.kMAXMotionPositionControl);
              Logger.recordOutput("at setpoint", liftPID.isAtSetpoint());
              isIntakeDeployed = true;
            },
            this)
        .until(() -> liftPID.isAtSetpoint())
        .beforeStarting(() -> Leds.color = CURRENTCOLOR.VIOLET)
        .finallyDo(
            () -> {
              Leds.color = CURRENTCOLOR.TEAM_COLOR;
              isIntakeDeployed = true;
            });
  }

  /* private boolean isAtSetpoint() {
    return liftMotor.getAbsoluteEncoder().getPosition() - liftPID.getSetpoint();
  } */

  public Command intakeRetract() {
    return Commands.run(
            () -> {
              liftPID.setSetpoint(
                  IntakeConstants.reverseLimit, ControlType.kMAXMotionPositionControl);
              Logger.recordOutput("at setpoint", liftPID.isAtSetpoint());

              isIntakeDeployed = false;
            },
            this)
        .until(() -> liftPID.isAtSetpoint())
        .beforeStarting(() -> Leds.color = CURRENTCOLOR.VIOLET)
        .finallyDo(
            () -> {
              Leds.color = CURRENTCOLOR.TEAM_COLOR;
              isIntakeDeployed = false;
            });
  }

  public Command toggleIntake() {
    return Commands.either(intakeRetract(), intakeDeploy(), () -> isIntakeDeployed());
  }

  public void intake() {
    wheelMotorLeft.set(IntakeConstants.intakeSpeed);
    wheelMotorRight.set(IntakeConstants.intakeSpeed);
  }

  public Command intakeCommand() {
    return Commands.run(
        () -> {
          intake();
        },
        this);
  }

  public void reverseIntake() {
    wheelMotorLeft.set(-IntakeConstants.intakeSpeed);
    wheelMotorRight.set(-IntakeConstants.intakeSpeed);
  }

  public Command reverseIntakeCommand() {
    return Commands.run(
        () -> {
          reverseIntake();
        },
        this);
  }

  public void stopIntake() {
    wheelMotorLeft.stopMotor();
    wheelMotorRight.stopMotor();
  }

  public Command stopIntakeWheelsCommand() {
    return Commands.runOnce(
        () -> {
          stopIntake();
        },
        this);
  }

  public void stopAllIntakeMotors() {
    wheelMotorLeft.stopMotor();
    wheelMotorRight.stopMotor();
    liftMotor.stopMotor();
  }

  public Command intakeManual(double power) {
    return Commands.run(() -> liftMotor.set(power));
  }

  public boolean isIntakeDeployed() {
    return isIntakeDeployed;
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Lift Position", liftMotor.getAbsoluteEncoder().getPosition());
    // SmartDashboard.putNumber("Lift PID Setpoint Position",
    // liftPID.getMAXMotionSetpointPosition());
    // SmartDashboard.putNumber("Lift PID Setpoint", liftPID.getSetpoint());
  }
}
