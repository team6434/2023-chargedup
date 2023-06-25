/** Copyright (c) FIRST and other WPILib contributors.
 * Open Source Software; you can modify and/or share it under the terms 
 *of the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// Camera Imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

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
  private Timer gameTimer;
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
  public double driveDirection = 1.0;
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
  // Vision (Limelight)
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ta = table.getEntry("ta");
  // PID Control
  public double leftSpeed;
  public double rightSpeed;
  public double steeringAdjust;
  public double distanceAdjust;
  // Auto
  private Autonomous auto;
  private String autoSelected;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String Turn = "Turn 90";
  private static final String defaultAuto = "Default";
  private static final String DeliverExit = "Deliver & Exit";
  private static final String secondRowCube = "Second Row Cube Delivery";
  private static final String driveOut = "Drive Out";
  private static final String pidAuto = "PID Control";  
  private static final String ChargeStationAuto = "Charge Station";
  private static final String DeliverOverChargeStation = "Deliver Over Charge Station";
  private static final String CommunityChargeStation = "Exit Community & Charge Station";
  private Timer autoTimer;
  //Test 
  private final SendableChooser<String> calChooser = new SendableChooser<>();
  private static final String CalDefault = "Default";
  private static final String minSpeed = "Minimum Speed";
  private static final String encoderCal = "Encoder Calibration";
  private final SendableChooser<String> encoderCalChooser = new SendableChooser<>();
  private static final String dis1 = "Encoder Calibration 1 Meter";
  private static final String dis3 = "Encoder Calibration 3 Meter";
  private static final String dis5 = "Encoder Calibration 5 Meter";
  private static final String dis8 = "Encoder Calibration 8 Meter";

  private double statethinggy = 0;
  
  private int state = 0;

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
    gameTimer = new Timer();
    arm = new Arm(this);
    intake = new Intake();
    intake.open();
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
    Shuffleboard.selectTab("General Infomation");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("Default Auto", defaultAuto);
    autoChooser.addOption("Turn 90", Turn);
    autoChooser.addOption("Deliver & Exit", DeliverExit);
    autoChooser.addOption("Sencond Row Cube Delivery", secondRowCube);
    autoChooser.addOption("Drive Out", driveOut);
    autoChooser.addOption("PID Control", pidAuto);
    autoChooser.addOption("Charge Station", ChargeStationAuto);
    autoChooser.addOption("Deliver Over Charge Station", DeliverOverChargeStation);
    autoChooser.addOption("Exit Community & Charge Station", CommunityChargeStation);
    //Cal
    SmartDashboard.putData("Calibration Chooser", calChooser);
    autoChooser.setDefaultOption("Default", CalDefault);
    autoChooser.addOption("Minimum Speed", minSpeed);
    autoChooser.addOption("Encoder Calibration", encoderCal);
    // Encoder Calibration
    SmartDashboard.putData("Encoder Cal Distance", encoderCalChooser);
    autoChooser.setDefaultOption("Encoder Calibration 1 Meter", dis1);
    autoChooser.addOption("Encoder Calibration 3 Meter", dis3);
    autoChooser.addOption("Encoder Calibration 5 Meter", dis5);
    autoChooser.addOption("Encoder Calibration 8 Meter", dis8);

    SmartDashboard.putData("Gyro", drivetrain.navx);
    SmartDashboard.putData("Drivebase", drivetrain.drive);
    SmartDashboard.putData("Left Encoder", drivetrain.leftEncoder);
    SmartDashboard.putData("Right Encoder", drivetrain.rightEncoder); 
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
    driveModes[driveMode] = "Arcade2";
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
    SmartDashboard.putString("Left Y Joystick",
        "LeftY = " + String.format("%.2f", controller.getLeftY()));
    SmartDashboard.putString("Left X Joystick",
        "LeftX = " + String.format("%.2f", controller.getLeftX()));
    SmartDashboard.putString("Right Y Joystick",
        "RightY = " + String.format("%.2f", controller.getRightY()));
    SmartDashboard.putString("Right X Joystick",
        "RightX = " + String.format("%.2f", controller.getRightX()));
    SmartDashboard.putNumber("Distance Encoder Value", drivetrain.distanceAVG());
    SmartDashboard.putString("Auto Timer",
        "Auto Timer = " + String.format("%.2f", autoTimer.get()));
    SmartDashboard.putNumber("Robot Pitch", drivetrain.robotPitch());
    SmartDashboard.putNumber("Robot Roll", drivetrain.robotRoll());
    SmartDashboard.putNumber("Neo Position", arm.armEncoder.getPosition());
    SmartDashboard.putNumber("Neo Velocity", -arm.armEncoder.getVelocity());
    SmartDashboard.putString("Game Time", 
        "" + gameTimer.get());
    SmartDashboard.putNumber("StateThinggy", statethinggy);
    SmartDashboard.putBoolean("Drive Slow", driveSlow);
    SmartDashboard.putBoolean("Drive Reverse", driveReverse);
    SmartDashboard.putNumber("AUTOPOWER", auto.autoPower);
    SmartDashboard.putNumber("State", state);
  }

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    Shuffleboard.selectTab("Autonomous");
    SmartDashboard.putNumber("Auto State", state);
    autoSelected = autoChooser.getSelected();
    System.out.println("Auto selected: " + autoSelected);
    autoTimer.reset();
    autoTimer.start();
    gameTimer.reset();
    gameTimer.start();
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
    if (autoTimer.get() < 1) {
      arm.moveArm(1, 1);
    } else {
      arm.armOff();
    }    
    switch (autoSelected) {
      case DeliverExit:
          if (state == 0) {
            if (drivetrain.leftEncoder.getDistance() > -0.4) {
              auto.driveStraight(-0.4);
            } else {
              state = 1;
              auto.driveOff();
            }
          } else if (state == 1) {
            if (drivetrain.leftEncoder.getDistance() < 3) {
              auto.driveStraight(0.9);
            } else {
              state = 2;
              auto.driveOff();
            }
          } else {
            auto.driveOff();
          }
        break;
      case driveOut:
        if (state == 0) {
          if (drivetrain.leftEncoder.getDistance() > -1.5) {
            auto.driveStraight(-0.5);
          } else {
            state = 1;
            auto.driveOff();
          }
        } else {
          state = 2;
          auto.driveOff();
        }
        break;
      case pidAuto:
        // Drive forward at 0.5 speed for 1 meter using gyro to stablize the heading
        if (state == 0) {
          if (drivetrain.leftEncoder.getDistance() < 1) {
            auto.driveStraight(0.3);
          } else {
            state = 1;
            auto.driveOff();
          }
        } else if (state == 1) {
          if (drivetrain.leftEncoder.getDistance() > 0) {
            auto.driveStraight(-0.3);
          } else {
            state = 2;
            auto.driveOff();
          }
        } else {
          auto.driveOff();
        }
        break;
      case ChargeStationAuto: 
        if (state == 0) {
          if (Math.abs(drivetrain.navx.getRoll()) < 9) {
            auto.driveStraight(1.0);
          } else {
            state = 1;
            auto.driveOff();
          }
        } else if (state == 1) {
          auto.chargeStation();
        } else {
          auto.driveOff();
        }
        break;
      case DeliverOverChargeStation:
      if (state == 0) {
        if (drivetrain.leftEncoder.getDistance() > -0.4) {
          auto.driveStraight(-0.4);
        } else {
          state = 1;
          auto.driveOff();
        }
      } else if (state == 1) {
        if (drivetrain.leftEncoder.getDistance() < 4) {
          auto.driveStraight(1);
        } else {
          state = 2;
          auto.driveOff();
        }
      } else if (state == 2) {
        if (Math.abs(drivetrain.navx.getRoll()) < 9) {
          auto.driveStraight(-1.0);
        } else {
          state = 3;
          auto.driveOff();
        }
      } else if (state == 3) {
        auto.chargeStation();
      } else {
        auto.driveOff();
      }
        break;
      case CommunityChargeStation:
        if (state == 0) { // Drive onto charge station.
          if (Math.abs(drivetrain.navx.getRoll()) < 2) { // Checks if the robot is flat on the floor.
            auto.driveStraight(-1);
          } else {
            state = 1;
            auto.driveOff();
          }
        } else if (state == 2) { // Gets to other side of charge station.
          if (Math.abs(drivetrain.navx.getRoll()) > 5) { 
            auto.driveStraight(-0.8);
          } else {
            state = 3;
            auto.driveOff();
          }
        } else if (state == 3) { // Charge Station code
          auto.chargeStation();
        } else {
          state = 3;
          auto.driveOff();
          autoTimer.stop();

        }
        break;
      case secondRowCube:
        if (state == 0) { // Close Pistons
          auto.driveOff();
          if (autoTimer.get() >= 1.0) {
            intake.close();
            state = 1;
          }
        } else if (state == 1) {
          auto.driveOff();
          if (autoTimer.get() >= 1.25) {
            state = 2;
          }
        } else if (state == 2) {
          auto.driveOff();
          if (arm.armEncoder.getPosition() <= 102) {
            arm.smoothArm(102.5);
          } else {
            state = 3;
          }
        } else if (state == 3) { // Close Pistons
          auto.driveOff();
          intake.open();
          state = 4;
        } else if (state == 4) {
          arm.smoothArm(7);
        }
        else {
          arm.armOff();
          autoTimer.stop();
          auto.driveOff();
        }
        break;
      case defaultAuto:
      default:
        arm.armOff();
        autoTimer.stop();
        break;
     }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Shuffleboard.selectTab("TeleOp");
    drivetrain.resetEncoder();
    drivetrain.resetGyro();
    gameTimer.start();
  }

  /** This function is called peri+odically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Auto balance
    if (controller.getRightTriggerAxis() == 1) {
      auto.chargeStation();
    // Limelight
    } else if (controller.getLeftBumper()) {
      double x = tx.getDouble(0.0);
      double area = ta.getDouble(0.0);
      double KpSteer; // 0.0255
      double KpDistance = 0.0375;
      
      table.getEntry("pipeline").setNumber(1);
      if (Math.abs(x) > 15) {
        KpSteer = 0.024; 
      } else {
        KpSteer = 0.039;
      }
      steeringAdjust = KpSteer * x;
      double distError = 10.0 - area;
      distanceAdjust = KpDistance * distError; // TO DO: FAR AWAY = BIG KP, CLOSE UP = SMALL KP
      double driveSpeed = 0; // -0.25

      leftSpeed = driveSpeed + steeringAdjust; // - distanceAdjust;
      rightSpeed = driveSpeed - steeringAdjust; //- distanceAdjust;

      drivetrain.drive.tankDrive(leftSpeed, rightSpeed);
    //drive
    } else { 
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
        armMovement = arm.smoothArm(2.0);
      } else if (armTest == "Delivery" ) {
        armMovement = arm.smoothArm(102); //100
      } else if (armTest == "PickUp" ) {
        armMovement = arm.smoothArm(100); // 120
      } else if (armTest == "Ground" ) {
        armMovement = arm.smoothArm(145); // 135
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
    Shuffleboard.selectTab("General Infomation");
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
  public void testInit() {
    autoTimer.start();
    // calSelected = calChooser.getSelected();
    // encoderCalSelected = encoderCalChooser.getSelected();
    // System.out.println("Cal selected: " + calSelected);
    // drivetrain.resetGyro();
    // drivetrain.resetEncoder();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // switch (calSelected) {
    //   case minSpeed:
    //     if (drivetrain.leftEncoder.getDistance() < 1) { 
    //       testSpeed += increaseSpeed;
    //       auto.driveStraight(testSpeed);
    //     } else {
    //       auto.driveOff();
    //     }
    //     break;
    //   case encoderCal:
    //     switch (encoderCalSelected) {
    //       case dis1:
    //         if (drivetrain.leftEncoder.getDistance() < 1) {
    //           auto.driveStraight(0.3);
    //         } else {
    //           auto.driveOff();
    //         }
    //         break;
    //       case dis3:
    //         if (drivetrain.leftEncoder.getDistance() < 3) {
    //           auto.driveStraight(0.3);
    //         } else {
    //           auto.driveOff();
    //         }
    //         break;
    //       case dis5:
    //         if (drivetrain.leftEncoder.getDistance() < 5) {
    //           auto.driveStraight(0.3);
    //         } else {
    //           auto.driveOff();
    //         }
    //         break;
    //       case dis8:
    //         if (drivetrain.leftEncoder.getDistance() < 8) {
    //           auto.driveStraight(0.3);
    //         } else {
    //           auto.driveOff();
    //         }
    //         break;
    //     }
    //     break;        
    //   case CalDefault:
    //   default:
    //     auto.driveOff();
    //     break;
    // }
  }

  /** This function is called once when simulation is started. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}