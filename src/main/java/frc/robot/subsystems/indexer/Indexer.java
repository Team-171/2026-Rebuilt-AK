package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class Indexer extends SubsystemBase {

  private final SparkMax motor =
      new SparkMax(IndexerConstants.motorId, SparkLowLevel.MotorType.kBrushless);

  private final SparkMaxConfig topMotorConfig = new SparkMaxConfig();

  public Indexer() {
    topMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IndexerConstants.stallCurrentLimit, IndexerConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(true);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void index() {
    motor.set(IndexerConstants.indexSpeed);
  }

  public void reverseIndex() {
    motor.set(-IndexerConstants.indexSpeed);
  }

  public Command reverseIndexerCommand() {
    return Commands.run(
        () -> {
          reverseIndex();
        },
        this);
  }

  public void stopIndex() {
    motor.stopMotor();
  }

  public Command shootWithOUTDelayCommand() {
    return Commands.run(
        () -> {
          index();
        },
        this);
  }

  public Command shootWithdelay() {
    return Commands.runOnce(
            () -> {
              motor.set((IndexerConstants.indexSpeed));
            },
            this)
        .beforeStarting(new WaitCommand(Constants.delaySeconds));
  }

  public Command stopIndexCommand() {
    return Commands.runOnce(
        () -> {
          stopIndex();
        },
        this);
  }

  @Override
  public void periodic() {}
}
