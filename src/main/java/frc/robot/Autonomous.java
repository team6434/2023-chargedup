package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
  private Drivetrain drivetrain;
  public double autoMaxPower = 0;
  public double driveSMin = 0.10;
  public double autoPower = 0;

  public Autonomous(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void drive(double speedLeft, double speedRight) {
    drivetrain.drive.tankDrive(speedLeft, speedRight, false);
  }

  public void driveStraight(double speed) {
    double error = 0 - drivetrain.robotBearing();
    double kP = 0.01;

    this.drive(speed + kP * error, speed - kP * error);
  }

  public void turn(double turnDegree, double turnSpeed) {
    double errorTurn = turnDegree - drivetrain.robotBearing();
    double kPTurn = 0.01456;
    
    this.drive(turnSpeed + kPTurn * errorTurn, -turnSpeed - kPTurn * errorTurn);
  }

  public void chargeStation() {
    double error = drivetrain.robotRoll();
    double kP = (1 - driveSMin) / 20;
    // double kP = 0.1;
    SmartDashboard.putNumber("Error", error);
    if (Math.abs(drivetrain.navx.getRoll()) < 2) {
      autoMaxPower = 0;
      autoPower = 0;
    } else {
      autoMaxPower = 0.6;
      autoPower = (driveSMin + kP * error) * autoMaxPower;
    }

    this.drive(autoPower, autoPower);
  }

  public void driveOff() {
    drivetrain.drive.tankDrive(0, 0);
  }  
}