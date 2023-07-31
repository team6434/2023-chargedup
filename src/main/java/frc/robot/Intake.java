package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Intake {

  public PWMVictorSPX intakeRollers;

  public Intake() {
    intakeRollers = new PWMVictorSPX(3);
    // intakeRollers.setInverted(false);
  }

  public void take(double speed) {
    intakeRollers.set(speed);
  }

  public void remove(double speed) {
    intakeRollers.set(-speed);
  }

  public void takeCone() {
    take(0.5);
  }

  public void removeCone() {
    remove(1.0);
  }

  public void intakeOff() {
    take(0.0);
  }
}
