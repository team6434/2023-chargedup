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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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
  public XboxController controller;

  private Drivetrain drivetrain;
  private Integer driveMode = 0;
  private final String[] driveModes = {
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
  // Arm
  private Arm arm;
  // Intake (Claw)
  private Intake intake;
  // NavX (Gyro)
  private boolean GyroReset = false;
  // Vision (Limelight)
  private Vision vision;
  public NetworkTable tableLimelight = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = tableLimelight.getEntry("tx");
  private NetworkTableEntry ty = tableLimelight.getEntry("ty");
  private NetworkTableEntry ta = tableLimelight.getEntry("ta");
  // PID Control
  public double leftSpeed;
  public double rightSpeed;
  public double steeringAdjust;
  public double distanceAdjust;
  // Auto
  private Autonomous auto;
  private Timer autoTimer;
  private Integer autoMode = 0;
  private final String[] autoModes = {
    "Disabled [DEFAULT]",
    "Drive out",
    "JoshAuto"
  };
  
  int state = 0;

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
    // arm = new Arm();
    intake = new Intake();
    vision = new Vision();
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
    vision.switchLED(0);
  }

  /** This function is called every 20 ms, no matter the mode.
   * Use this for items like diagnostics.
   */
  @Override
  public void robotPeriodic() {
    if (controller.getAButtonPressed()) {
      if (GyroReset == false) {
        GyroReset = true;
        drivetrain.resetGyro();
        GyroReset = false;
      } else {
        GyroReset = false;
      }
    }


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
          driveRotate = -controller.getRightX() * driveRmax * driveDirection;
        } else {
          driveRotate = controller.getRightX() * driveRmax * driveDirection;
        }
        driveSpeed = -controller.getLeftY() * driveSmax * driveDirection;
        SmartDashboard.putString("Drive Speed",
            "driveSpeed    = " + String.format("%.2f", driveSpeed));
        SmartDashboard.putString("Drive Rotate",
            "driveRotate = " + String.format("%.2f", driveRotate));
        break;
      case "Arcade1":
      case "Curvature1":
        if (driveDirection >= 0 ) {
          driveRotate = -controller.getLeftX() * driveRmax * driveDirection;
        } else {
          driveRotate = controller.getLeftX() * driveRmax * driveDirection;
        }
        driveSpeed = -controller.getLeftY() * driveSmax * driveDirection;
        SmartDashboard.putString("Drive Speed",
            "driveSpeed    = " + String.format("%.2f", driveSpeed));
        SmartDashboard.putString("Drive Rotate",
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
      SmartDashboard.putString("Drive Left",
            "driveLeft = " + String.format("%.2f", driveLeft));
        SmartDashboard.putString("Drive Right",
            "driveRight  = " + String.format("%.2f", driveRight));
    }
    /** Driver station: Basic Tab */
    SmartDashboard.putString("Left Y Joystick",
        "LeftY = " + String.format("%.2f", controller.getLeftY()));
    SmartDashboard.putString("Left X Joystick",
        "LeftX = " + String.format("%.2f", controller.getLeftX()));
    SmartDashboard.putString("Right Y Joystick",
        "RightY = " + String.format("%.2f", controller.getRightY()));
    SmartDashboard.putString("Right X Joystick",
        "RightX = " + String.format("%.2f", controller.getRightX()));
    SmartDashboard.putString("Drive Direction",
        "driveDirection = " + String.format("%.2f", driveDirection));
    SmartDashboard.putString("Robot Angle",
        "Robot Angle = " + String.format("%.2f", drivetrain.robotBearing()));
    SmartDashboard.putString("Drivetrain Left Encoder", 
        "Left Encoder" + String.format("%.3f", drivetrain.left_Encoder.getDistance()));
    SmartDashboard.putString("Drivetrain Right Encoder",
        "Right Encoder = " + String.format("%.3f", drivetrain.right_Encoder.getDistance()));
    SmartDashboard.putString("Area", // Shows area of camera taken up by part to the camera.
        "Area = " + String.format("%.3f", ta));
    SmartDashboard.putString("Y", // Shows the vertical location of the object to the camera.
        "Y = " + String.format("%.3f", ty));
    SmartDashboard.putString("X", // Shows the horizontal location of the object to the camera.
        "X = " + String.format("%.3f", tx));
    SmartDashboard.putBoolean("Drive Slow", driveSlow);
    SmartDashboard.putBoolean("Drive Reverse", driveReverse);
  }

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    System.out.println("Auto: RUNNING  > " + autoMode);
    autoTimer.reset();
    autoTimer.start();
    drivetrain.resetEncoder();
    drivetrain.resetGyro();
    state = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoModes[autoMode]) {
      case "JoshAuto":
        if (drivetrain.right_Encoder.getDistance() > -767 && state == 0) {
          SmartDashboard.putNumber("Right Encoder", drivetrain.right_Encoder.getDistance());
          auto.driveStraight(-0.4);
          SmartDashboard.putString("Auto Text", "Driving Straight");
        } 
        else if (drivetrain.robotBearing() < 90 && state == 1) {    
          state = 1; 
          // SmartDashboard.putString("Auto Text", "Spin!");
          auto.drive(-0.2, 0.2);
        } else if (drivetrain.left_Encoder.getDistance() > 100) {
          state = 2;
          SmartDashboard.putNumber("State", state);

          // arm.raiseArm(0.5);
        } else {
          // SmartDashboard.putString("Auto Text", "Turning off");
        }
      case "Drive out":
        if (drivetrain.right_Encoder.getDistance() > auto.driveDistance(3)) {
          auto.driveStraight(0.5);
        }
        break;
      case "Disabled [DEFAULT]":
      default:
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    drivetrain.resetEncoder();
    drivetrain.resetGyro();
  }

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
    // Intake (Pnuematics)
    if (controller.getRightTriggerAxis() > 0.75) {
      intake.open();
    }
    if (controller.getLeftTriggerAxis() > 0.75) {
      intake.close();
    }
    // Arm
    if (controller.getAButtonPressed()) {
      arm.cubePickUp(0.5);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    SmartDashboard.putStringArray("Auto List", autoModes);
  }

  @Override
  public void disabledPeriodic() {
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

    if (controller.getRightTriggerAxis() > 0.75) {
      autoMode++;
      autoMode = autoMode % 3;
      System.out.println("autoMode num.  > " + String.valueOf(autoMode));
      System.out.println("Drive: SELECTED > " + autoModes[autoMode]);
    }
    if (controller.getLeftTriggerAxis() > 0.75) {
      autoMode--;
      if (autoMode < 0) {
        autoMode = 2;
      }
      System.out.println("Drivemode num.  > " + String.valueOf(autoMode));
      System.out.println("Drive: SELECTED > " + autoModes[autoMode]);      
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

/** TO DO:
 * 
 * 001. 2023_2021_HOOKA robotPeriodic() testing code to be REMOVED
 *      (or moved to testPeriodic)
 * 
 * 002. Review Shuffleboard code testing and incorporate into project.
 * 
 * 003. Review TO DO for implementation of Shuffleboard in all active 
 *      robot codebases.
 * 
 * 004. Review default driverstation vs Shuffleboard and identify 
 *      cabailbities desirable in Shuffleboard. Add to TO DO.
 * 
 * 009. 2023_2021_HOOKA Review Robot.java TO DO list and merge here.
 * 
 * 010. 2023_2021_HOOKA Review encoder comment and correct inforation
 *      for correct encoder type and details.
 * 011. 2023_2021_HOOKA Confirm display of encoder data on Shuffleboard.
 *      Review encoder data available and include any desirable data on
 *      Shuffleboard. See WPILib reference library.
 * 012. 2023_2021_HOOKA Detailed encoder test to interpret encoder data.
 * 013. 2023_2021_HOOKA Robot.java can public double driveEncoderScaleMM = 478.778720;
 *      by converted to private double driveEncoderScaleMM = 478.778720;
 * 014. 2023_2021_HOOKA Review Shuffleboard content and comment detail (per 011).
 * 015. 2023_chargedup apply encoder learning above and perform encoder
 *      testing.
  *  
 * 020. 2023_2021_HOOKA Confirm display of gyro data on Shuffleboard.
 *      Review gyro data available and include any desirable data on
 *      Shuffleboard. See WPILib reference library.
 * 021. Hooker detailed gyro test to interpret gyro data.
 *      Review Shuffleboard content and comment detail (per 021).
 * 022. Apply to 2023_chargedup and perform gyro testing.
 * 
 * 030. Reflect and identify uses of drivetrain encoder data (add to TO DO).
 * 031. Reflect and identify uses of gyro data (add to TO DO).
 *
 *
 * 
 * 080. Calibrate the minimum drivetrain power required to start moving.
 *      Use encoders &/or gyro to confirm motion.
 * 
 * 081. Create drivetrain motion profile.
 *      Set power range from minimum drive power (per 080 above) to
 *      maximum power (see 082 below).
 *      Future drivetrain control and potentially all motor control
 *      should incorporate motion profiling to smooth the control of the
 *      robot and help protect the robot from the driver and itself.
 *      https://www.chiefdelphi.com/t/motion-profiling/115133
 * 
 * 082. Create calibration for driveXmax/driveYmax.
 * 
 * 083. Create calibaration routine for motion profiling.
 * 
 * 090. Camera - can a 2nd camera be switched automatically in drivestation
 *      based on driving direction.
 * 
 */