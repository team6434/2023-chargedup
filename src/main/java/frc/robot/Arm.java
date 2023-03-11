package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
  public CANSparkMax CANarmSpark;
  public RelativeEncoder armEncoder;
  public double armSpeed = 0.2;

  // private final double topPointEncoderValue = 180;
  // private final double cubePickUpPoint = 270;

  public Arm(Robot robot) {
    CANarmSpark = new CANSparkMax(10, MotorType.kBrushless);
    CANarmSpark.setIdleMode(IdleMode.kBrake);
    CANarmSpark.setSmartCurrentLimit(10);
    armEncoder = CANarmSpark.getEncoder();
  }

  public void raiseArm(double armSpeed) {
      CANarmSpark.set(armSpeed);
  }

  // Open point val: 143.52
  // Close point val: 5

  public void lowerArm(double armSpeed) {
    CANarmSpark.set(-armSpeed);
  }

  public void armOff() {
    CANarmSpark.set(Math.abs(0));
  }

  public boolean moveArm(double destAngle, double armSpeed) {
    double direction;
    double tolerance = 5.0;
    double maxTol = destAngle + tolerance;
    double minTol = destAngle - tolerance;
    if (armEncoder.getPosition() < destAngle) {
      direction = +1.0;
    } else {
      direction = -1.0; 
    }   
    if (armEncoder.getPosition() > maxTol || armEncoder.getPosition() < minTol ) {
      CANarmSpark.set(armSpeed * direction); 
      return true;
    } else if (destAngle == 0.0) {
       return false;
    } else {
      CANarmSpark.set(0.0); 
      return true;
    }
  }

  public boolean smoothArm(double destAngle) {
    double armMaxSpeed = 0.4;
    double targetVel = 700;
    double toleranceVel = 200;
    double armdirection;
    double toleranceAngle = 5.0;
    double maxAngle = destAngle + toleranceAngle;
    double minAngle = destAngle - toleranceAngle;
    double curAngle = armEncoder.getPosition();
    double curVel = armEncoder.getVelocity();

    if (curAngle < destAngle) {
      armdirection = +1.0;
    } else {
      armdirection = -1.0; 
    }
    double maxVel = (targetVel + toleranceVel) * armdirection;
    double minVel = (targetVel - toleranceVel) * armdirection;
    if (armdirection > 0 ) {
      if (maxVel > curVel) armSpeed -= 0.0005;
      if (minVel < curVel) armSpeed += 0.0005;
    }
    if (armdirection < 0 ) {
      if (maxVel < curVel) armSpeed -= 0.0005;
      if (minVel > curVel) armSpeed += 0.0005;
    }
    if (Math.abs(armSpeed) > armMaxSpeed ) armSpeed = armMaxSpeed;
    SmartDashboard.putNumber("Cur. Angle", curAngle);
    SmartDashboard.putNumber("Dest. Angle", destAngle);
    SmartDashboard.putNumber("Arm Speed", armSpeed);
    SmartDashboard.putNumber("Arm Dir", armdirection);
    SmartDashboard.putNumber("Arm Vel", curVel);
    SmartDashboard.putNumber("Min Vel.", minVel);
    SmartDashboard.putNumber("Max Vel.", maxVel);
    if (curAngle > maxAngle || curAngle < minAngle ) {
      CANarmSpark.set(armSpeed * armdirection); 
      return true;
    } else if (destAngle == 0.0) {
      armSpeed = 0.2;
      return false;
    } else {
      armSpeed = 0.2;
      CANarmSpark.set(0.0); 
      return true;
    }
  }
  
  public boolean openArm(double armSpeed) {
    if (armEncoder.getPosition() < 135) {
      CANarmSpark.set(armSpeed); 
      return true;
    } else {
       return false;
    }
  }

  public boolean closeArm(double armSpeed) {
    if (armEncoder.getPosition() > 2) {
      CANarmSpark.set(-armSpeed); 
      return true;
    } else {
       return false;
    }
  }

  // TODO Write openArm() with angle
}
