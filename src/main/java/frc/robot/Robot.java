/** Copyright (c) FIRST and other WPILib contributors.
 * Open Source Software; you can modify and/or share it under the terms 
 *of the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Camera Imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

// // Network Tables
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

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
  private Boolean armMovement = false;
  private String armTest = "Ground";
  // Intake (Claw)
  private Intake intake;
  // Limelight
  // public NetworkTable tableLimelight = NetworkTableInstance.getDefault().getTable("limelight");
  // private Vision vision;
  // private NetworkTableEntry tx = tableLimelight.getEntry("tx");
  // private NetworkTableEntry ta = tableLimelight.getEntry("ta");
  // PID Control
  public double leftSpeed;
  public double rightSpeed;
  public double steeringAdjust;
  public double distanceAdjust;
  // Auto
  private Autonomous auto;
  
  private String autoSelected;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  private static final String deliverExit = "Deliver & Exit";
  private static final String defaultAuto = "Default";
  private static final String JoshAuto = "JoshAuto";
  private static final String driveOut = "Drive Out";
  
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
    autoChooser.addOption("Deliver & Exit", deliverExit);
    autoChooser.addOption("JoshAuto", JoshAuto);
    autoChooser.addOption("Drive Out", driveOut);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    arm = new Arm(this);
    // SmartDashboard.putString("ArmSpeed",
    //   "ArmSpeed      = " + String.format("%.2f", arm.armSpeed));
    intake = new Intake();
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
  }

  /** This function is called every 20 ms, no matter the mode.
   * Use this for items like diagnostics.
   */
  @Override
  public void robotPeriodic() {
    if (controller.getPOV() == 0) {
      controller.setRumble(RumbleType.kBothRumble, 0.2);
    } else if (controller.getPOV() == 90) {
      controller.setRumble(RumbleType.kBothRumble, 0.4);
    } else if (controller.getPOV() == 180) {
      controller.setRumble(RumbleType.kBothRumble, 0.6);
    } else if (controller.getPOV() == 270) {
      controller.setRumble(RumbleType.kBothRumble, 0.8);
    } else {
      controller.setRumble(RumbleType.kBothRumble, 0);
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
        "Left Encoder = " +  String.format("%.3f", drivetrain.leftEncoder.getDistance()));
    SmartDashboard.putString("Drivetrain Right Encoder",
        "Right Encoder = " + String.format("%.3f", drivetrain.rightEncoder.getDistance()));
    SmartDashboard.putString("Distance Encoder Value",
        "Distance Encoder Value = " + String.format("%.3f", drivetrain.distanceAVG()));
    SmartDashboard.putString("Auto Timer",
        "Auto Timer = " + String.format("%.2f", autoTimer.get()));
    SmartDashboard.putString("Auto Selected",
        "Auto Selected = " + autoSelected);
    SmartDashboard.putString("Robot Angle", // TODO: Work on robot angle
        "Robot Angle = " + String.format("%.2f", drivetrain.robotBearing()));
    // // Neo data (ARM)
    // SmartDashboard.putString("NEO Encoder",
    //     "NEO Encoder = " + String.format("%.2f", arm.armEncoder.getPosition()));
    // SmartDashboard.putString("NEO Velocity",
    //     "NEO Velocity = " + String.format("%.2f", -arm.armEncoder.getVelocity()));
    SmartDashboard.putBoolean("Drive Slow", driveSlow);
    SmartDashboard.putBoolean("Drive Reverse", driveReverse);
  }

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    // Shuffleboard.selectTab("Autonomous");
    autoSelected = autoChooser.getSelected();
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
    
    // auto.driveOff();
    switch (autoSelected) {
      case deliverExit:
        if (drivetrain.robotBearing() <= 90) { // Turn in direction closer to 90 degrees.
        auto.turnRight(0.15);
        } else if (drivetrain.robotBearing() >= 91) {
          auto.turnLeft(0.15);
        } else {
          drivetrain.drive.tankDrive(0, 0);
          autoTimer.stop();
        }
        break;
        // if(drivetrain.leftEncoder.getDistance() < -0.3 && state == 0) {
        //   drivetrain.drive.tankDrive(-0.5, -0.5);
        // } else if (drivetrain.leftEncoder.getDistance() < 1.5 && state == 1) {
        //   state = 1;
        //   drivetrain.drive.tankDrive(0.5, 0.5);
        // } 
      case JoshAuto:
          if(drivetrain.leftEncoder.getDistance() < 1.5) {
            // drivetrain.drive.tankDrive(0.5, 0.5);
            auto.driveStraight(0.5);
          }else {
            drivetrain.drive.tankDrive(0, 0);
          }
        // if (drivetrain.distanceAVG() <= 1.0) && state == 0) {
        //   auto.driveStraight(0.2);
        // }
        // else if (drivetrain.distanceAVG() <= -0.3 && state == 1) {
        //   state = 1;
        //   auto.driveStraight(-0.2);
        // }
        // else if (drivetrain.navx.getAngle() <= 90 && state == 2) { // Turn in direction closer to 90 degrees.
        //   state = 2;
        //   if (drivetrain.navx.getAngle() > 90) {
        //     auto.turnLeft(0.2);
        //   }
        //   else if (drivetrain.navx.getAngle() < 90) {
        //     auto.turnRight(0.2);
        //   }
        //   else {
        //     auto.driveOff();
        //   }
        // } 
        break;
      case driveOut:
        if(state == 0 && drivetrain.leftEncoder.getDistance() > -0.3) {
          drivetrain.drive.tankDrive(-0.5, -0.5);
        } else if (state == 1 && drivetrain.leftEncoder.getDistance() < 0.3) {
          state = 1;
          drivetrain.drive.tankDrive(0.5, 0.5);
        } else {
          state = 2;
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
    // Shuffleboard.selectTab("Tele Op");
    drivetrain.resetEncoder();
    drivetrain.resetGyro();
  }

  /** This function is called peri+odically during operator control. */
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
    // double x = tx.getDouble(0.0);
    // double area = ta.getDouble(0.0);
    // double KpSteer; // 0.0255
    // double KpDistance = 0.0375;

    // if (controller.getRightBumper()) {
    //   tableLimelight.getEntry("pipeline").setNumber(2);
    //   if (Math.abs(x) > 18) {
    //     KpSteer = 0.025; 
    //   } else {
    //     KpSteer = 0.0375;
    //   }
    //   steeringAdjust = KpSteer * x;
    //   double distError = 10.0 - area;
    //   distanceAdjust = KpDistance * distError; // TO DO: FAR AWAY = BIG KP, CLOSE UP = SMALL KP
    //   double driveSpeed = 0; // -0.25

    //   leftSpeed = driveSpeed + steeringAdjust; // - distanceAdjust;
    //   rightSpeed = driveSpeed - steeringAdjust; //- distanceAdjust;

    //   SmartDashboard.putString("Left Speed", // Test if dashboard is working.
    //       "Left Speed = " + leftSpeed);
    //   SmartDashboard.putString("Right Speed", // Test if dashboard is working.
    //       "Right Speed = " + rightSpeed);
    //   SmartDashboard.putString("Distance Adjust", // Test if dashboard is working.
    //       "Dist Adjust= " + distanceAdjust);
    //   SmartDashboard.putString("Distance Error", // Test if dashboard is working.
    //       "Dist Error= " + distError);

    //   drivetrain.drive.tankDrive(leftSpeed, rightSpeed);
    // }
    // Intake (Pnuematics)
    if (controller.getRightBumperPressed()) {
      intake.togglePiston();
    }
    // Arm movement
    if (controller.getAButtonPressed()) {
      armTest = "Home";
      if (armMovement == false) armMovement = true;
    }
    if (controller.getBButtonPressed()) {
      armTest = "Delivery";
      if (armMovement == false) armMovement = true;
    }
    if (controller.getYButtonPressed()) {
      armTest = "Ground";
      if (armMovement == false) armMovement = true;
    }
    if (controller.getXButtonPressed()) {
      armTest = "PickUp";
      if (armMovement == false) armMovement = true;
    }
    if (armMovement) {
      if (armTest == "Home" ) {
        armMovement = arm.smoothArm(0.0);
      } else if (armTest == "Delivery" ) {
        armMovement = arm.smoothArm(100.0);
      } else if (armTest == "PickUp" ) {
        armMovement = arm.smoothArm(120.0);
      } else if (armTest == "Ground" ) {
        armMovement = arm.smoothArm(135.0);
      } else {
        arm.armOff();
        armMovement = false;
      }
    } 
    if (!armMovement) {
      arm.armOff();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
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