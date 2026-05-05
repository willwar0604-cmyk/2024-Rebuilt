package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public SparkMax leftHotwheel =
      new SparkMax(Constants.LEFT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  public SparkMax rightHotwheel =
      new SparkMax(Constants.RIGHT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  public SparkMax loadingDrum = new SparkMax(Constants.LOADING_DRUM, SparkMax.MotorType.kBrushless);
  public SparkFlex groundPickup =
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
    return runOnce(
        () -> {
          groundPickup.set(0.8);
          leftHotwheel.set(0.8);
          rightHotwheel.set(0.8);
          loadingDrum.set(0.8);
        });
  }

  public Command load() {
    return runOnce(() -> loadingDrum.set(0.8));
  }

  public Command stop() {
    return runOnce(
        () -> {
          groundPickup.stopMotor();
          leftHotwheel.stopMotor();
          rightHotwheel.stopMotor();
          loadingDrum.stopMotor();
        });
  }

  public void periodic() {}
}
