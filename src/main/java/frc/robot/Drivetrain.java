package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain {

  public PWMVictorSPX driveLeft, driveRight;
  public DifferentialDrive drive;
  public AHRS navx;

  public Drivetrain() {
    navx = new AHRS(SPI.Port.kOnboardCS0); // kOnboardCS"Number" is for the NavX Micro for NavX2 Select kMXP
    navx.calibrate();
    navx.reset();

    driveLeft = new PWMVictorSPX(0);
    driveLeft.setInverted(false);
    driveRight = new PWMVictorSPX(1);
    driveRight.setInverted(true);

    drive = new DifferentialDrive(driveLeft, driveRight);
    drive.setDeadband(0.02);
    drive.setMaxOutput(1.0);
  }
}
