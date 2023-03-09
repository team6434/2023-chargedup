package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm {
  public CANSparkMax CANarmSpark;
  public RelativeEncoder armEncoder;
  // private final double topPointEncoderValue = 180;
  // private final double cubePickUpPoint = 270;

  public Arm() {
    CANarmSpark = new CANSparkMax(10, MotorType.kBrushless);
    CANarmSpark.setIdleMode(IdleMode.kCoast);
    CANarmSpark.setSmartCurrentLimit(10);
    armEncoder = CANarmSpark.getEncoder();
  }

  public void raiseArm(double armSpeed) {
    CANarmSpark.set(armSpeed);
  }

  public void lowerArm(double armSpeed) {
    CANarmSpark.set(-armSpeed);
  }

  public void armOff() {
    CANarmSpark.set(Math.abs(0));
  }
}
