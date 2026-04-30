package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.aux.*;

public class AuxCommands extends SubsystemBase {
  private final Intake intake;
  private final Shooter shooter;
  private final LoadingDrum loadingDrum;

  private final DigitalInput irSensor = new DigitalInput(1);

  public AuxCommands(Intake intake, Shooter shooter, LoadingDrum loadingDrum) {
    this.intake = intake;
    this.shooter = shooter;
    this.loadingDrum = loadingDrum;
  }

  // intakes untill ir sensor is triggered, then spins shooter motors (ideally)
  public void intakeTrue() {
    if (irSensor.get()) {
      intake.intakeRun();
      loadingDrum.drumRun();
    } else {
      intake.intakeStop();
      loadingDrum.drumStop();
      shooter.shooterRun();
    }
  }

  public void intakeFalse() {
    intake.intakeStop();
    loadingDrum.drumStop();
  }

  public void shootTrue() {
    loadingDrum.drumRun();
    shooter.shooterRun();
  }

  public void shootFalse() {
    loadingDrum.drumStop();
    shooter.shooterStop();
  }

  public Command autoIntakeTrue() {
    return run(this::intakeTrue);
  }
}
