package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class Shooter extends SubsystemBase {

  /* private final SparkMax shooterMotor
  = new SparkMax(ShooterConstants.shooterCanId, MotorType.kBrushless); */
  private final TalonFX talonMotor = new TalonFX(ShooterConstants.talonId, CANBus.roboRIO());
  private final TalonFX talonMotor2 = new TalonFX(ShooterConstants.talonId2, CANBus.roboRIO());

  private static final TalonFXConfiguration talonConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(40)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive));

  private static final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private static final Follower follower = new Follower(33, MotorAlignmentValue.Opposed);
  private static final InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

  private final Drive drive;

  /* private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController motorPID = shooterMotor.getClosedLoopController(); */
  public Shooter(Drive drive) {
    this.drive = drive;
    /* motorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.stallCurrentLimit, ShooterConstants.freeCurrentLimit)
            .voltageCompensation(12)
            .inverted(true);
    motorConfig.closedLoop
            .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
            .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    motorConfig.closedLoop.maxMotion
            .cruiseVelocity(ShooterConstants.cruiseVelocity)
            .maxAcceleration(ShooterConstants.maxAcceleration)
            .allowedProfileError(ShooterConstants.allowedError);
    motorConfig.encoder
            .positionConversionFactor(ShooterConstants.conversionFactor)
            .velocityConversionFactor(ShooterConstants.conversionFactor);

    SparkUtil.tryUntilOk(
            shooterMotor,
            5,
            ()
            -> shooterMotor.configure(
                    motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)); */

    var slot0Configs = talonConfig.Slot0;
    slot0Configs.kV = 0.15; // velocity feedforward 0.25
    slot0Configs.kS = 0.2; // static feedforward 0.2
    slot0Configs.kA = 0.0;
    slot0Configs.kP = 0.5;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    var motionMagic = talonConfig.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 180;
    motionMagic.MotionMagicAcceleration = 360;
    motionMagic.MotionMagicJerk = 4000;

    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonMotor.getConfigurator().apply(talonConfig);
    talonMotor2.getConfigurator().apply(talonConfig);

    treeMap.put(1.7, 42.0);
    treeMap.put(2.0, 44.0);
    treeMap.put(2.35, 47.0);
    treeMap.put(3.0, 49.0);
    treeMap.put(3.25, 51.0);
    treeMap.put(3.88, 54.0);
    treeMap.put(4.4, 57.0);
  }

  public Command shooterVelocity(double vel) {
    return Commands.run(
        () -> {
          talonMotor.setControl(velocityRequest.withVelocity(vel));
          talonMotor2.setControl(follower);
          Logger.recordOutput("Shooter Velocity", vel);
        },
        this);
  }

  public void shoot(double speed) {
    // motorPID.setSetpoint(ShooterConstants.shootSpeed, ControlType.kMAXMotionVelocityControl);
    // TODO write wrapper for auto aiming
    Logger.recordOutput("Shooter/ConstSpeed", mapped(speed));
    // shooterMotor.set(mapped(speed));
    talonMotor.set(mapped(speed));
    talonMotor2.setControl(follower);
  }

  public Command shootWithDistanceCommand() {
    return Commands.run(
        () -> {
          talonMotor.setControl(velocityRequest.withVelocity(treeMap.get(getDisToHub())));
          talonMotor2.setControl(follower);
          Logger.recordOutput("Shooter Velocity", treeMap.get(getDisToHub()));
        },
        this);
  }

  public static double mapped(double value) {
    return MathUtil.clamp((value + 1) / 2.0, 0.0, 1.0);
  }

  public void stopShooter() {
    // shooterMotor.stopMotor();
    talonMotor.stopMotor();
  }

  public Command stopShooterCommand() {
    return Commands.runOnce(
        () -> {
          talonMotor.stopMotor();
          talonMotor2.stopMotor();
        },
        this);
  }

  public double getDisToHub() {
    Pose2d hubPose =
        new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), new Rotation2d());
    return PhotonUtils.getDistanceToPose(drive.getPose(), AllianceFlipUtil.apply(hubPose));
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Shooter/Velocity", shooterMotor.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/TalonVelocity", talonMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/distance", getDisToHub());
  }

  public Command test() {
    return Commands.run(
        () -> {
          talonMotor.setControl(velocityRequest.withVelocity(50.0));
          talonMotor2.setControl(follower);
        },
        this);
  }
}
