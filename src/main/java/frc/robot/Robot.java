/** Copyright (c) FIRST and other WPILib contributors.
 * Open Source Software; you can modify and/or share it under the terms 
 *of the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** Camera Imports */
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
// Network Tables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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
  public double driveRmax = 0.75;
  private double driveSpeed;
  public double driveSmax = 1.00;
  private double driveCurveMod = 0.70; // Curve drive is too powerful.
  /** Drive reverse controls*/
  public double driveDirection = -1.0;
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
  // // Vision (Limelight)
  // private Vision vision;
  // public NetworkTable tableLimelight = NetworkTableInstance.getDefault().getTable("limelight");
  // private NetworkTableEntry tx = tableLimelight.getEntry("tx");
  // private NetworkTableEntry ty = tableLimelight.getEntry("ty");
  // private NetworkTableEntry ta = tableLimelight.getEntry("ta");
  // PID Control
  public double leftSpeed;
  public double rightSpeed;
  public double steeringAdjust;
  public double distanceAdjust;
  // Auto
  private Autonomous auto;
  private static final String defaultAuto = "Default";
  private static final String JoshAuto = "JoshAuto";
  private static final String driveOut = "Drive Out";
  private String autoSelected;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  private Timer autoTimer;
  
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
    autoChooser.setDefaultOption("Default Auto", defaultAuto);
    autoChooser.addOption("JoshAuto", JoshAuto);
    autoChooser.addOption("Drive Out", driveOut);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    arm = new Arm();
    intake = new Intake();
    /* USB Camera
    // vision = new Vision();
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
    // vision.switchLED(0);
    */
  }

  /** This function is called every 20 ms, no matter the mode.
   * Use this for items like diagnostics.
   */
  @Override
  public void robotPeriodic() {
    // if (controller.getAButtonPressed()) {
    //   if (GyroReset == false) {
    //     GyroReset = true;
    //     drivetrain.resetGyro();
    //     GyroReset = false;
    //   } else {
    //     GyroReset = false;
    //   }
    // }
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
      //TODO: Replace formula below with drivetrain.driveSpeedTank() but requires testing
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
    // Driver station: Basic Tab
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
        "Left Encoder = " + String.format("%.3f", drivetrain.leftEncoder.getDistance()));
    SmartDashboard.putString("Drivetrain Right Encoder",
        "Right Encoder = " + String.format("%.3f", drivetrain.rightEncoder.getDistance()));
    SmartDashboard.putString("Distance Encoder Value",
        "Distance Encoder Value = " + String.format("%.3f", drivetrain.distanceAVG()));
    SmartDashboard.putString("Auto Timer",
        "Auto Timer = " + String.format("%.2f", autoTimer.get()));
    SmartDashboard.putString("Robot Angle", // TODO: Work on robot angle
        "Robot Angle = " + String.format("%.2f", drivetrain.navx.getAngle()));
    SmartDashboard.putString("NEO Encoder",
        "NEO Encoder = " + String.format("%.2f", arm.armEncoder.getPosition()));
    // SmartDashboard.putString("Area", // Shows area of camera taken up by part to the camera.
    //     "Area = " + String.format("%.3f", ta));
    // SmartDashboard.putString("Y", // Shows the vertical location of the object to the camera.
    //     "Y = " + String.format("%.3f", ty));
    // SmartDashboard.putString("X", // Shows the horizontal location of the object to the camera.
    //     "X = " + String.format("%.3f", tx));
    SmartDashboard.putBoolean("Drive Slow", driveSlow);
    SmartDashboard.putBoolean("Drive Reverse", driveReverse);
  }

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    autoSelected = autoChooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
    System.out.println("Auto selected: " + autoSelected);
    autoTimer.reset();
    autoTimer.start();
    drivetrain.resetEncoder();
    drivetrain.resetGyro();
    state = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
        /** *********************** NOTE ******************************
     * During both teleopPeriodic and autonomousPeriodic the drive
     * train needs to be updated regularly (every loop/period).
     * If the drivetrain is not updated often enough an error will be 
     * reported.
     * 
     *  DifferentialDrive... output not updated often enough.
     */
    switch (autoSelected) {
      case JoshAuto:
        driveSpeed = drivetrain.driveSpeedTank(0.2);
        if (drivetrain.distanceAVG() >= auto.driveDistance(-3)) {
          auto.driveStraight(driveSpeed);
        } else if (drivetrain.robotBearing() < 90) {
          auto.turnLeft(driveSpeed);
        } else if (drivetrain.robotBearing() < 0) {
          auto.turnRight(driveSpeed);
        } else {
          auto.driveOff();
          autoTimer.stop();
        }
        break;
      case driveOut:
        if (autoTimer.get() < 5.0) {
          auto.driveStraight(0.2);
        } else {
          auto.driveOff();
          autoTimer.stop();
        }
        break;
      case defaultAuto:
      default:
        auto.driveOff();
        autoTimer.stop();
        break;
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
    /** *********************** NOTE ******************************
     * During both teleopPeriodic and autonomousPeriodic the drive
     * train needs to be updated regularly (every loop/period).
     * If the drivetrain is not updated often enough an error will be 
     * reported.
     * 
     *  DifferentialDrive... output not updated often enough.
     */
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
    if (controller.getAButton()) {
      arm.raiseArm(0.4);
    } else if (controller.getBButton()) {
      arm.lowerArm(0.4);
    } else {
      arm.armOff();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    /*if (controller.getRightBumperPressed()) {
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
    }*/
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