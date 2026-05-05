package frc.robot.subsystems.aux;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TeamSwitch extends SubsystemBase {
  public DigitalInput teamSwitch = new DigitalInput(2);

  public boolean getTeamColor() {
    return teamSwitch.get();
  }
}

// still dunno what to do with this