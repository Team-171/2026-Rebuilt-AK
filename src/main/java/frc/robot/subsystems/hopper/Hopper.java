package frc.robot.subsystems.hopper;

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

public class Hopper extends SubsystemBase {

  private final SparkMax motor =
      new SparkMax(HopperConstants.motorId, SparkLowLevel.MotorType.kBrushless);

  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  public Hopper() {
    motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(HopperConstants.stallCurrentLimit, HopperConstants.freeCurrentLimit)
        .voltageCompensation(12)
        .inverted(true);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void moveHopper() {
    motor.set(HopperConstants.motorSpeed);
  }

  public void stopHopper() {
    motor.stopMotor();
  }

  public Command stopHopperCommand() {
    return Commands.runOnce(
        () -> {
          stopHopper();
        },
        this);
  }

  public Command shootWithOUTDelay() {
    return Commands.run(
        () -> {
          moveHopper();
        },
        this);
  }

  public Command shootWithDelay() {
    return Commands.runOnce(
            () -> {
              motor.set(HopperConstants.motorSpeed);
            },
            this)
        .beforeStarting(new WaitCommand(Constants.delaySeconds));
  }

  @Override
  public void periodic() {}

  public Command reverseHopperCommand() {
    return Commands.run(
        () -> {
          motor.set(-HopperConstants.motorSpeed);
        },
        this);
  }
}
