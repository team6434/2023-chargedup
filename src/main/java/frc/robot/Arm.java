package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm {
  public CANSparkMax CANarmSpark;
  public RelativeEncoder armEncoder;
  private final double topPointEncoderValue = 180;
  private final double cubePickUpPoint = 270;

  public Arm() {
    CANarmSpark = new CANSparkMax(1, MotorType.kBrushless);
    CANarmSpark.setIdleMode(IdleMode.kBrake);
    CANarmSpark.setSmartCurrentLimit(10);
    armEncoder = CANarmSpark.getEncoder();
  }

  public void cubePickUp(double armSpeed) {
    CANarmSpark.set(Math.abs(armSpeed));
    if (armEncoder.getPosition() != topPointEncoderValue) {
      CANarmSpark.set(Math.abs(-0.2));
    } else if (armEncoder.getPosition() != cubePickUpPoint) {
      CANarmSpark.set(Math.abs(-0.4));
    }
  }
}
