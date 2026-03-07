package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

  private static TalonFXConfiguration talonConfig =
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

  private static final MotionMagicVelocityVoltage shooterRequest =
      new MotionMagicVelocityVoltage(0);

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
    slot0Configs.kV = 0.12; // velocity feedforward
    slot0Configs.kS = 0.0; // static feedforward
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    talonMotor.getConfigurator().apply(talonConfig);

    var motionMagic = talonConfig.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = ShooterConstants.mmCV;
    motionMagic.MotionMagicAcceleration = ShooterConstants.mmA;
    motionMagic.MotionMagicJerk = ShooterConstants.mmJ;

    // TODO: must find relationship between distance from target and velocity
    treeMap.put(1.526, 11.0);
    treeMap.put(1.82, 10.5);
    treeMap.put(2.16, 9.5);
    treeMap.put(2.5, 9.0);
    treeMap.put(2.82, 8.6);
    treeMap.put(3.135, 8.2);
    treeMap.put(3.5, 7.8);
    treeMap.put(3.85, 7.5);
    treeMap.put(4.29, 7.2);
  }

  public void shooterVelocity(double vel) {
    // motorPID.setSetpoint(vel, ControlType.kMAXMotionVelocityControl);
    talonMotor.setControl(shooterRequest.withVelocity(vel));
  }

  public void shoot(double speed) {
    // motorPID.setSetpoint(ShooterConstants.shootSpeed, ControlType.kMAXMotionVelocityControl);
    // TODO write wrapper for auto aiming
    Logger.recordOutput("Shooter/ConstSpeed", mapped(speed));
    // shooterMotor.set(mapped(speed));
    talonMotor.set(mapped(speed));
  }

  public void shootWithDistance() {
    talonMotor.setControl(shooterRequest.withVelocity(treeMap.get(getDisToHub())));
  }

  public static double mapped(double value) {
    return MathUtil.clamp((value + 1) / 2.0, 0.0, 1.0);
  }

  public void stopShooter() {
    // shooterMotor.stopMotor();
    talonMotor.stopMotor();
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
  }
}
