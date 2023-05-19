package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake {
  public DoubleSolenoid intakeDoubleSolenoid;
  public boolean pistonToggle = false;

  public Intake() {
    intakeDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  }

  public void open() {
    intakeDoubleSolenoid.set(Value.kForward);
  }

  public void close() {
    intakeDoubleSolenoid.set(Value.kReverse);
  }

  public void off() {
    intakeDoubleSolenoid.set(Value.kOff);
  }

  public void togglePiston() {
    if (pistonToggle == true) {
      this.open();
      pistonToggle = false;
    } else if (pistonToggle == false) {
      this.close();
      pistonToggle = true;
    }
  }
}
