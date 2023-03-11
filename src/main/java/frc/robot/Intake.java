package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake {
  public DoubleSolenoid intakeDoubleSolenoid;

  public Intake() {
    intakeDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }

  public void open() {
    intakeDoubleSolenoid.set(Value.kForward);
  }

  public void close() {
    intakeDoubleSolenoid.set(Value.kReverse);
  }

  public void togglePiston() {
    intakeDoubleSolenoid.toggle();
  }
}
