package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.aux.Intake;
import frc.robot.subsystems.aux.LoadingDrum;

public class IntakeCommands extends SubsystemBase {
  private final Intake intake;
  private final LoadingDrum loadingDrum;
  private final DigitalInput irSensor = new DigitalInput(1);

  public IntakeCommands(Intake intake, LoadingDrum loadingDrum) {
    this.intake = intake;
    this.loadingDrum = loadingDrum;
  }

  public void runTrue() {
    intake.intakeRun();
    loadingDrum.drumRun();
  }

  public void runFalse() {
    intake.intakeStop();
    loadingDrum.drumStop();
  }
}
