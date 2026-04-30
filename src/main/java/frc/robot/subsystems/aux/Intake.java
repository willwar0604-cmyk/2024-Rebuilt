package frc.robot.subsystems.aux;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;

public class Intake extends SubsystemBase {
  boolean run;

  SparkMax leftHotwheel =
      new SparkMax(Constants.IntakeConstants.LEFT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  SparkMax rightHotwheel =
      new SparkMax(Constants.IntakeConstants.RIGHT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  SparkFlex groundPickup =
      new SparkFlex(Constants.IntakeConstants.GROUND_PICKUP, SparkFlex.MotorType.kBrushless);

  public Intake() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);

    SparkFlexConfig pickupConfig = new SparkFlexConfig();
    pickupConfig.idleMode(IdleMode.kBrake);

    leftHotwheel.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightHotwheel.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    groundPickup.configure(
        pickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeRun() {
    groundPickup.set(PowerConstants.INTAKE_POWER);
    leftHotwheel.set(PowerConstants.INTAKE_POWER);
    rightHotwheel.set(-PowerConstants.INTAKE_POWER);
  }

  public void intakeStop() {
    groundPickup.set(0);
    leftHotwheel.set(0);
    rightHotwheel.set(0);
  }

  public void periodic() {}
}
