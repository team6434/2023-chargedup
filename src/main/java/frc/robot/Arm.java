package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Arm {
  private PWMSparkMax armSpark;
  private CANSparkMax CANarmSpark;

  public Arm() {
    armSpark = new PWMSparkMax(4);
    armSpark.setInverted(false);

    CANarmSpark = new CANSparkMax(10, MotorType.kBrushless);
    CANarmSpark.setIdleMode(IdleMode.kBrake);
    CANarmSpark.setSmartCurrentLimit(10);
    CANarmSpark.getEncoder().getPosition();
    CANarmSpark.getEncoder().getVelocity();
    CANarmSpark.getOutputCurrent();
  }

  public void raiseArm(double armSpeed) {
    armSpark.set(Math.abs(armSpeed));
    CANarmSpark.set(0.5);
  }
}
