package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class Spindexer extends SubsystemBase {

  private final SparkMax motor =
      new SparkMax(SpindexerConstants.motorId, SparkLowLevel.MotorType.kBrushless);

  private final SparkMaxConfig topMotorConfig = new SparkMaxConfig();

  public Spindexer() {
    topMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(
            SpindexerConstants.stallCurrentLimit, SpindexerConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(false);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void index() {
    motor.set(SpindexerConstants.indexSpeed);
  }

  public void stopIndex() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {}
}
