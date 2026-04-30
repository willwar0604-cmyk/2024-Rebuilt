package frc.robot.subsystems.aux;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;

public class LoadingDrum extends SubsystemBase {
  SparkMax loadingDrum =
      new SparkMax(Constants.DrumConstants.LOADING_DRUM, SparkMax.MotorType.kBrushless);

  SparkClosedLoopController drumController = loadingDrum.getClosedLoopController();

  public LoadingDrum() {
    SparkMaxConfig drumConfig = new SparkMaxConfig();
    drumConfig.idleMode(IdleMode.kBrake);

    loadingDrum.configure(
        drumConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void drumRun() {
    loadingDrum.set(PowerConstants.DRUM_POWER);
  }

  public void drumStop() {
    loadingDrum.set(0);
  }
}
