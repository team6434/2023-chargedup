package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

public class Drivetrain {

  public PWMVictorSPX driveLeft, driveRight;
  public DifferentialDrive drive;
  public AHRS navx;

  public Drivetrain() {
    // Gyro Code
    // navx = new AHRS(SPI.Port.kOnboardCS0); // kOnboardCS"Number" is for the NavX Micro for NavX2 Select kMXP.
    navx = new AHRS(I2C.Port.kOnboard); // NavX Micro
    // navx = new AHRS(I2C.Port.kMXP);  // NavX2
    navx.calibrate(); // Calibrates the gyro then resets the yaw, pitch, and roll to zero.
    navx.reset();

    /*TO DO:
     *  - Test if NavX works
     *  - Test if New NavX Code works 
     *  - Use Hooker code
     */
    
    driveLeft = new PWMVictorSPX(0);
    driveLeft.setInverted(false);
    driveRight = new PWMVictorSPX(1);
    driveRight.setInverted(true);

    drive = new DifferentialDrive(driveLeft, driveRight);
    drive.setDeadband(0.02);
    drive.setMaxOutput(1.0);
  }
}
  