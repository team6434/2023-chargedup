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
  public Encoder left_Encoder;
  public Encoder right_Encoder;

  public Drivetrain() {
    // Gyro Code
    navx = new AHRS(I2C.Port.kOnboard); // NavX Micro
    // navx = new AHRS(I2C.Port.kMXP);  // NavX2
    navx.calibrate(); // Calibrates the gyro then resets the yaw, pitch, and roll to zero.
    navx.reset();
    // Encoder Code
    left_Encoder = new Encoder(0, 1);
    right_Encoder = new Encoder(2, 3);

    driveLeftFront = new PWMVictorSPX(0);
    driveLeftBack = new PWMVictorSPX(1);
    driveLeft = new MotorControllerGroup(driveLeftFront, driveLeftBack);
    driveLeft.setInverted(false);

    driveRightFront = new PWMVictorSPX(2);
    driveRightBack = new PWMVictorSPX(3);
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
  public void resetEncoder() {
    left_Encoder.reset();
    right_Encoder.reset();
  }
  // Reads gyro (between 0-360)a
  public double robotBearing()  {
    return navx.getAngle() % 360;
  }
}
  