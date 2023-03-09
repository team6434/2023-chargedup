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
    drive(speed, speed);
  }

  public void turnRight(double speed) {
    drive(speed, -speed);
  }

  public void turnLeft(double speed) {
    drive(-speed, speed);
  }

  public double driveDistance(double distanceM) {
    return 4308 * distanceM;
  }

  public void driveOff() {
    drivetrain.drive.tankDrive(0, 0);
  }
}