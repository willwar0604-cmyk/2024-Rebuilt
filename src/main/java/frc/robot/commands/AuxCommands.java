package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
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
  public void intakeSystemRun() {
    if (irSensor.get()) {
      intake.intakeOn();
      loadingDrum.drumOn();
    } else {
      intake.intakeOff();
      loadingDrum.drumOff();
      shooter.shooterOn();
    }
  }

  public void intakeSystemStop() {
    intake.intakeOff();
    loadingDrum.drumOff();
  }

  public void shooterSystemRun() {
    loadingDrum.drumOn();
    shooter.shooterOn();
  }

  public void shooterSystemStop() {
    loadingDrum.drumOff();
    shooter.shooterOff();
  }
}
