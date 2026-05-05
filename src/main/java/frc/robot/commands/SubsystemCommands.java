package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;

public class SubsystemCommands extends SubsystemBase {
  public DigitalInput irSensor = new DigitalInput(1);

  private final Intake intake;
  private final Shooter shooter;

  public SubsystemCommands(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
  }

  //   For some reason i have a hunch the sensor output is gonna be opposite of what i want
  //
  //   public boolean irSensorFlip(boolean status) {
  //     if (irSensor.get()) {
  //         return status = false;
  //     } else {
  //         return status = true;
  //     }
  //   }

  public Command waitForLoad() {
    return new ParallelCommandGroup(intake.stop(), shooter.spinUp().onlyIf(() -> irSensor.get()));
  }

  public Command end() {
    return new ParallelCommandGroup(shooter.stop(), intake.stop());
  }

  public Command intake() {
    return intake.intake().onlyWhile(() -> irSensor.get()).finallyDo(() -> this.waitForLoad());
  }

  public Command shoot() {
    return intake.load().onlyWhile(() -> shooter.isUpToSpeed()).finallyDo(() -> this.end());
  }
}
