package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;

public class Drivetrain {
  private Robot robot;

  public PWMVictorSPX driveLeftFront, driveLeftBack, driveRightFront, driveRightBack;
  public MotorControllerGroup driveLeft, driveRight;
  public DifferentialDrive drive;
  public AHRS navx;
  public Encoder leftEncoder;
  public Encoder rightEncoder;
  public double distancePerPulse = 0.00023328;  //20230427 calibrated @ BPHS M1 over 1m
  
  public Drivetrain(Robot robot) {
    // Gyro Code
    navx = new AHRS(SPI.Port.kMXP);
    navx.calibrate(); // Calibrates the gyro then resets the yaw, pitch, and roll to zero.
    navx.reset();
    // Encoder Code
    leftEncoder = new Encoder(9, 8);
    leftEncoder.setReverseDirection(true);
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder = new Encoder(1, 2);
    rightEncoder.setReverseDirection(false);
    rightEncoder.setDistancePerPulse(distancePerPulse);

    driveLeftFront = new PWMVictorSPX(9);
    driveLeftBack = new PWMVictorSPX(8);
    driveLeft = new MotorControllerGroup(driveLeftFront, driveLeftBack);
    driveLeft.setInverted(false);

    driveRightFront = new PWMVictorSPX(0);
    driveRightBack = new PWMVictorSPX(1);
    driveRight = new MotorControllerGroup(driveRightFront, driveRightBack);
    driveRight.setInverted(true);

    drive = new DifferentialDrive(driveLeft, driveRight);
    drive.setDeadband(robot.driveDeadband);
    drive.setMaxOutput(1.0);
  }

  public double driveSpeedTank(double speed) {
      return speed * robot.driveSmax * robot.driveDirection;
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
  public double robotBearing() {
    return navx.getAngle() % 360;
  }
  
  public double robotPitch() {
    return navx.getPitch() % 360;
  }

  public double robotRoll() {
    return navx.getRoll() % 360;
  }
}
  