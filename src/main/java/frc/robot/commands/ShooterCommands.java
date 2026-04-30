package frc.robot.commands;

import frc.robot.subsystems.aux.LoadingDrum;
import frc.robot.subsystems.aux.Shooter;

public class ShooterCommands {
  private final Shooter shooter;
  private final LoadingDrum loadingDrum;

  public ShooterCommands(Shooter shooter, LoadingDrum loadingDrum) {
    this.shooter = shooter;
    this.loadingDrum = loadingDrum;
  }

  public void startShoot() {
    shooter.shooterRun();
    loadingDrum.drumRun();
  }

  public void stopShoot() {
    shooter.shooterStop();
    loadingDrum.drumStop();
  }
}
