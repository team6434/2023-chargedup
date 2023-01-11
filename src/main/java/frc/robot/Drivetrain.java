package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Drivetrain {

  public PWMVictorSPX driveLeft, driveRight;
  public DifferentialDrive drive;

  public Drivetrain() {
    driveLeft = new PWMVictorSPX(0);
    driveLeft.setInverted(false);
    driveRight = new PWMVictorSPX(1);
    driveRight.setInverted(true);

    drive = new DifferentialDrive(driveLeft, driveRight);
    drive.setDeadband(0.02);
    drive.setMaxOutput(1.0);
  }
}
