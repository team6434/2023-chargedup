package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Encoder;

public class Drivetrain {

  public PWMVictorSPX driveLeftFront, driveLeftBack, driveRightFront, driveRightBack;
  public MotorControllerGroup driveLeft, driveRight;
  public DifferentialDrive drive;
  public AHRS navx;
  public Encoder leftEncoder;
  public Encoder rightEncoder;

  public Drivetrain() {
    // Gyro Code
    navx = new AHRS(I2C.Port.kOnboard); // NavX Micro
    navx = new AHRS(I2C.Port.kMXP);  // NavX2
    navx.calibrate(); // Calibrates the gyro then resets the yaw, pitch, and roll to zero.
    navx.reset();
    // Encoder Code
    leftEncoder = new Encoder(9, 8);
    rightEncoder = new Encoder(1, 2);

    driveLeftFront = new PWMVictorSPX(9);
    driveLeftBack = new PWMVictorSPX(8);
    driveLeft = new MotorControllerGroup(driveLeftFront, driveLeftBack);
    driveLeft.setInverted(false);

    driveRightFront = new PWMVictorSPX(0);
    driveRightBack = new PWMVictorSPX(1);
    driveRight = new MotorControllerGroup(driveRightFront, driveRightBack);
    driveRight.setInverted(true);

    drive = new DifferentialDrive(driveLeft, driveRight);
    drive.setDeadband(0.02);
    drive.setMaxOutput(1.0);
  }
  
  // Resets Gyro
  public void resetGyro() {
    navx.reset();
  }
  
  // Resets Encoder
  public void resetEncoder() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // Finds average encoder distannce
  public double distanceAVG() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
  }
  
  // Reads gyro (between 0-360)
  public double robotBearing()  {
    return navx.getAngle() % 360;
  }
}
  