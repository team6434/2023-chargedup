package frc.robot;

public class Autonomous {
  private Drivetrain drivetrain;

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

  public void driveOff() {
    drivetrain.drive.tankDrive(0, 0);
  }  
}