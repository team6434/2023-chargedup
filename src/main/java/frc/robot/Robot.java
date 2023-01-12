/** Copyright (c) FIRST and other WPILib contributors.
 * Open Source Software; you can modify and/or share it under the terms 
 *of the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** Camera Imports */
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import java.util.Objects;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/** The VM is configured to automatically run this class.
 * It calls the functions corresponding to each mode, as described in
 * TimedRobot documentation. If you change the name of this class or the
 * package after creating this project, you must also update the
 * build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  private XboxController controller;

  private Drivetrain drivetrain;
  private Integer driveMode = 0;
  private String[] driveModes = {
    "Arcade2",
    "Arcade1",
    "Curvature2",
    "Curvature1",
    "Tank"
  };
  private boolean driveSlow = false;
  /** Drive power: tank drive */
  private double driveLeft;
  private double driveRight;
  /** Drive power: all other drive modes */
  private double driveRotate;
  private double driveRmax = 0.75;
  private double driveSpeed;
  private double driveSmax = 1.00;
  private double driveCurveMod = 0.70; // Curve drive is too powerful.
  /** Drive reverse controls*/
  private double driveDirection = +1.0;
  private double driveDirectionInit;
  private double driveReverseTimeout = 2.0;
  private boolean driveReverse = false;
  private double driveTime;
  private Timer driveTimer;
  // Auto
  private Autonomous auto;
  private Timer autoTimer;
  private String autoMode;
  private String autoModeNew;
    private String[] autoModes = {
    "Disabled [DEFAULT]",
    "DS1",
    "DS2",
    "DS3",
    "DS1C",
    "DS2C",
    "DS3C",
    "Drive out"
  };
  
  Thread visionThread;

  /** Runs when the robot is first started up.
   * It should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    controller = new XboxController(0);
    drivetrain = new Drivetrain();
    driveTimer = new Timer();
    auto = new Autonomous(drivetrain);
    autoTimer = new Timer();
    visionThread = new Thread(
      () -> {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo(
          "Rectangle",
          640,
          480
        );
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }
          Imgproc.rectangle(
            mat,
            new Point(250, 100),
            new Point(400, 400),
            new Scalar(255, 255, 255),
            5
          );
          outputStream.putFrame(mat);
        }
      }
    );
    visionThread.setDaemon(true);
    visionThread.start();
  }

  /** This function is called every 20 ms, no matter the mode.
   * Use this for items like diagnostics.
   */
  @Override
  public void robotPeriodic() {
    if (controller.getStartButtonPressed()) {
      if (driveSlow) {
        driveSlow = false;
        driveRmax = 0.75;
        driveSmax = 1.00;
      } else {
        driveSlow = true;
        driveRmax = 0.55;
        driveSmax = 0.6;
      }
    }
    if (controller.getBackButtonPressed()) {
      driveReverse = true;
      driveTimer.reset();
      driveTimer.start();
      /** NOTE: changing direction too fast will result in decay
       * of driveDirectionInit until button mashing stops.
       */
      driveDirectionInit = driveDirection;
    }
    if (driveReverse) {
      driveTime = driveTimer.get();
      if (driveTime < driveReverseTimeout) {
        /** Formulae test data:
         * https://docs.google.com/spreadsheets/d/1rYx2XxR7Iog9g2TUoHyRlVOy-5UTdpuhn5ZGh3oRd8g/edit#gid=0
         */
        driveDirection = driveDirectionInit *
            (driveReverseTimeout / 2 - driveTime) /
            (driveReverseTimeout / 2);
      } else {
        driveReverse = false;
        /** Ensure we reset driveDirection to max. -1.0 or +1.0
         * once driver stops pressing button.
         */
        if (driveDirectionInit <= 0) {
          driveDirection = +1.0;
        } else {
          driveDirection = -1.0;
        }
      }
    }
    /** NOTE: forward on the joystick produces a negative value.
     * Take care to invert the Y values from the joystick.
     */
    switch (driveModes[driveMode]) {
      case "Arcade2":
      case "Curvature2":
        if (driveDirection >= 0 ) {
          driveRotate = controller.getRightX() * driveRmax * driveDirection;
        } else {
          driveRotate = -controller.getRightX() * driveRmax * driveDirection;
        }
        driveSpeed = -controller.getLeftY() * driveSmax * driveDirection;
        SmartDashboard.putString("DB/String 0", "driveSpeed    = "
            + String.format("%.2f", driveSpeed));
        SmartDashboard.putString("DB/String 5", "driveRotation = "
            + String.format("%.2f", driveRotate));
        break;
      case "Arcade1":
      case "Curvature1":
        if (driveDirection >= 0 ) {
          driveRotate = controller.getLeftX() * driveRmax * driveDirection;
        } else {
          driveRotate = -controller.getLeftX() * driveRmax * driveDirection;
        }
        driveSpeed = -controller.getLeftY() * driveSmax * driveDirection;
        SmartDashboard.putString("DB/String 0",
            "driveSpeed    = " + String.format("%.2f", driveSpeed));
        SmartDashboard.putString("DB/String 5",
            "driveRotate = " + String.format("%.2f", driveRotate));
        break;
      case "Tank":
      default:
        if (driveDirection >= 0 ) {
          driveLeft = -controller.getLeftY() * driveSmax * driveDirection;
          driveRight = -controller.getRightY() * driveSmax * driveDirection;
        } else {
          driveLeft = -controller.getRightY() * driveSmax * driveDirection;
          driveRight = -controller.getLeftY() * driveSmax * driveDirection;
        }
      SmartDashboard.putString("DB/String 0",
            "driveLeft = " + String.format("%.2f", driveLeft));
        SmartDashboard.putString("DB/String 5",
            "driveRight  = " + String.format("%.2f", driveRight));
    }
    /** Driver station: Basic Tab */
    SmartDashboard.putString("DB/String 1",
        "LeftY = " + String.format("%.2f", controller.getLeftY()));
    SmartDashboard.putString("DB/String 2",
        "LeftX = " + String.format("%.2f", controller.getLeftX()));
    SmartDashboard.putString("DB/String 6",
        "RightY = " + String.format("%.2f", controller.getRightY()));
    SmartDashboard.putString("DB/String 7",
        "RightX = " + String.format("%.2f", controller.getRightX()));
    SmartDashboard.putString("DB/String 4",
        "driveDirection = " + String.format("%.2f", driveDirection));
        SmartDashboard.putString("DB/String 9",
        "driveMode = " + driveModes[driveMode]);
    SmartDashboard.putBoolean("DB/LED 0", driveSlow);
    SmartDashboard.putBoolean("DB/LED 1", driveReverse);
  }

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    autoMode = SmartDashboard.getString("Auto Selector",
      "Disabled [DEFAULT]");
    System.out.println("Auto: RUNNING  > " + autoMode);
    autoTimer.reset();
    autoTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoMode) {
      case "DS1":
        if (autoTimer.get() < 1.5) {
          auto.driveStraight(0.25);
        } else {
          auto.driveOff();
        }
        break;
      case "DS2":
        if (autoTimer.get() < 1.5) {
          auto.driveStraight(-0.25);
        } else {
          auto.driveOff();
        }
        break;
      case "DS3":
        if (autoTimer.get() < 3.0) {
          auto.drive(0.25, -0.25);
        } else {
          auto.driveOff();
        }
        break;
      case "DS1C":
        if (autoTimer.get() < 3.0) {
          auto.drive(-0.25, 0.25);
        } else {
          auto.driveOff();
        }
        break;
      case "DS2C":
        if (autoTimer.get() < 3.0) {
          auto.drive(-0.25, -0.25);
        } else {
          auto.driveOff();
        }
        break;
      case "DS3C":
        if (autoTimer.get() < 3.0) {
          auto.drive(0.25, 0.25);
        } else {
          auto.driveOff();
        }
        break;
      case "Drive out":
        break;
      case "Disabled [DEFAULT]":
      default:
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    switch (driveModes[driveMode]) {
      case "Arcade2":
      case "Arcade1":
        drivetrain.drive.arcadeDrive(
          driveSpeed,
          driveRotate,
          true
        );
        break;
      case "Curvature2":
        /** Curvature drive with a given forward and turn rate +
         * as well as a button for turning in-place.
         */
        drivetrain.drive.curvatureDrive(
          driveSpeed * driveCurveMod,
          driveRotate * driveCurveMod,
          controller.getRightStickButton()
        );
        break;
      case "Curvature1":
        /** Curvature drive with a given forward and turn rate +
         * as well as a button for turning in-place.
         */
        drivetrain.drive.curvatureDrive(
          driveSpeed * driveCurveMod,
          driveRotate * driveCurveMod,
          controller.getLeftStickButton()
        );
        break;
      case "Tank":
      default:
        drivetrain.drive.tankDrive(
          driveLeft,
          driveRight,
          true
        );
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    SmartDashboard.putStringArray("Auto List", autoModes);
  }

  @Override
  public void disabledPeriodic() {
    autoModeNew = SmartDashboard.getString("Auto Selector",
      "Drive out [DEFAULT]");      
    if (!Objects.equals(autoModeNew, autoMode)) {
      autoMode = autoModeNew;
      System.out.println("Auto: SELECTED > " + autoMode);

    }
    if (controller.getRightBumperPressed()) {
      driveMode++;
      driveMode = driveMode % 5;
      System.out.println("Drivemode num.  > " + String.valueOf(driveMode));
      System.out.println("Drive: SELECTED > " + driveModes[driveMode]);
    }
    if (controller.getLeftBumperPressed()) {
      driveMode--;
      if (driveMode < 0) {
        driveMode = 4;
      }
      System.out.println("Drivemode num.  > " + String.valueOf(driveMode));
      System.out.println("Drive: SELECTED > " + driveModes[driveMode]);      
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when simulation is started. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}