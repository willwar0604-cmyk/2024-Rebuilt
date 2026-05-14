package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final DigitalInput irSensor = new DigitalInput(0);

  private final SparkMax leftHotwheel =
      new SparkMax(Constants.LEFT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  private final SparkMax rightHotwheel =
      new SparkMax(Constants.RIGHT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  private final SparkMax loadingDrum =
      new SparkMax(Constants.LOADING_DRUM, SparkMax.MotorType.kBrushless);
  private final SparkFlex groundPickup =
      new SparkFlex(Constants.GROUND_PICKUP, SparkFlex.MotorType.kBrushless);

  public Intake() {
    SparkFlexConfig pickupConfig = new SparkFlexConfig();
    pickupConfig.idleMode(IdleMode.kBrake);

    groundPickup.configure(
        pickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);

    leftHotwheel.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightHotwheel.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    loadingDrum.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command intake() {
    return run(() -> {
          runAll(0.8);
        })
        .onlyWhile(() -> irSensor.get())
        .finallyDo(() -> stopAll());
  }

  public Command dump() {
    return run(() -> {
          runAll(-0.4);
        })
        .finallyDo(() -> stopAll());
  }

  public Command feed() {
    return run(() -> loadingDrum.set(0.7)).finallyDo(() -> stopAll());
  }

  public void runAll(double power) {
    groundPickup.set(power);
    leftHotwheel.set(power);
    rightHotwheel.set(-power);
    loadingDrum.set(power - 0.1);
  }

  public void stopAll() {
    groundPickup.stopMotor();
    leftHotwheel.stopMotor();
    rightHotwheel.stopMotor();
    loadingDrum.stopMotor();
  }

  public void periodic() {
    Logger.recordOutput("Intake/Note Loaded", !irSensor.get());
  }
}
