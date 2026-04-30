package frc.robot.subsystems.aux;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TeamSwitch extends SubsystemBase {
  public DigitalInput teamSwitch = new DigitalInput(2);
  public String sendSwitchData;

  public boolean getTeamColor() {
    sendSwitchData = !teamSwitch.get() ? "Red" : "Blue";
    SmartDashboard.putString("Which Team", sendSwitchData);
    return !teamSwitch.get(); // true if red, false if blue
  }
}
