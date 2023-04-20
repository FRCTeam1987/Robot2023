// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.PositionConfigs.*;
import static frc.robot.subsystems.wrist.Wrist.ANGLE_STRAIGHT;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.pneumatics.Pneumatics;
import frc.lib.team3061.pneumatics.PneumaticsIO;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PositionConfig;
import frc.robot.Constants.PositionConfigs;
import frc.robot.commands.*;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.auto.AutoPathHelper;
import frc.robot.commands.auto.AutoScoreSequenceNoHome;
import frc.robot.commands.auto.Balance;
import frc.robot.commands.auto.PreBalance;
import frc.robot.commands.wrist.HomeWrist;
import frc.robot.configs.CompRobotConfig;
import frc.robot.configs.TestRobotConfig;
import frc.robot.subsystems.MultiLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.claw.ClawIOSparkMAX;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOTalonSRX;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private RobotConfig config;
  private Drivetrain drivetrain;
  private Wrist wrist;
  private Claw claw;
  private Height height = Height.HIGH;
  private MultiLimelight multiLimelight;
  private boolean shouldUseVision = false;

  private boolean doubleSubstation = false;

  public enum Height {
    HIGH,
    MEDIUM,
    FLOOR,
    NONE
  }

  private Vision vision;
  private Arm arm;
  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // RobotContainer singleton
  private static final RobotContainer robotContainer = new RobotContainer();
  private final Map<String, Command> autoEventMap = new HashMap<>();
  XboxController driverController =
      new XboxController(0); // Creates a CommandXboxController on port 1.
  XboxController coDriverController = new XboxController(1);
  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023_TEST:
        case ROBOT_2023_COMP:
        case ROBOT_DEFAULT:
          {
            if (Constants.getRobot() == Constants.RobotType.ROBOT_2023_TEST) {
              config = new TestRobotConfig();
            } else if (Constants.getRobot() == Constants.RobotType.ROBOT_2023_COMP) {
              config = new CompRobotConfig();
            } else {
              config = new TestRobotConfig();
            }

            GyroIO gyro = new GyroIOPigeon2(0);

            int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
            int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
            int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
            double[] steerOffsets = config.getSwerveSteerOffsets();

            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        driveMotorCANIDs[0],
                        steerMotorCANDIDs[0],
                        steerEncoderCANDIDs[0],
                        steerOffsets[0]),
                    0,
                    config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        driveMotorCANIDs[1],
                        steerMotorCANDIDs[1],
                        steerEncoderCANDIDs[1],
                        steerOffsets[1]),
                    1,
                    config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        driveMotorCANIDs[2],
                        steerMotorCANDIDs[2],
                        steerEncoderCANDIDs[2],
                        steerOffsets[2]),
                    2,
                    config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        driveMotorCANIDs[3],
                        steerMotorCANDIDs[3],
                        steerEncoderCANDIDs[3],
                        steerOffsets[3]),
                    3,
                    config.getRobotMaxVelocity());

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            // new Pneumatics(new PneumaticsIORev()); // Needs CTRE for practice bot
            wrist = new Wrist(new WristIOTalonSRX(config.getWristRotatorID()));
            claw = new Claw(new ClawIOSparkMAX(config.getClawMotorID()));
            claw.setDefaultCommand(new DefaultClawRollersSpin(claw));
            multiLimelight =
                new MultiLimelight(
                    () ->
                        (Math.abs(drivetrain.getCharacterizationVelocity()) < 0.025
                            // velocity can accept pose
                            && shouldUseVision), // see how high this can go
                    "limelight-fl",
                    "limelight-fr");
            // temp
            // vision =
            //     new Vision(
            //         new VisionIOLimelight(
            //             "limelight-fl", "limelight-fr", "limelight-bl", "limelight-br"));

            arm =
                new Arm(
                    new ArmIOTalonFX(
                        config.getArmLeaderMotorID(),
                        config.getArmFollowerMotorID(),
                        config.getArmCanCoderID(),
                        config.getArmTelescopeID(),
                        config.getArmPotentiometerAnalogId(),
                        config.getCANBusName()));
            break;
          }
        case ROBOT_SIMBOT:
          {
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, config.getRobotMaxVelocity());
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            new Pneumatics(new PneumaticsIO() {});
            // AprilTagFieldLayout layout;
            // try {
            //   layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            // } catch (IOException e) {
            //   layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            // }
            // new Vision(
            //     new VisionIOSim(
            // layout,
            // drivetrain::getPose,
            // RobotConfig.getInstance().getRobotToCameraTransform()));

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, config.getRobotMaxVelocity());

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, config.getRobotMaxVelocity());

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, config.getRobotMaxVelocity());

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, config.getRobotMaxVelocity());
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      new Pneumatics(new PneumaticsIO() {});
      // new Vision(new VisionIO() {});
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            () -> 1,
            () -> {
              // if (claw.getGamePiece() == GamePiece.CONE && driverController.getAButtonPressed())
              // {
              //   return 180;
              // }
              return driverController.getPOV();
            },
            driverController::getLeftStickButtonPressed));

    configureButtonBindings();
    configureAutoCommands();
    configureSmartDashboard();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all the buttons to commands.
   */
  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  private void configureSmartDashboard() {
    // ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    // debugTab.add("Arm Length (inches)", arm.getArmLength()).withSize(2, 2).withPosition(0, 0);
    // debugTab.add("Arm Angle (Degrees)", arm.getArmAngle()).withSize(2, 2).withPosition(2, 0);

    // TAB_MAIN.add(
    //     "Full Balance",
    //     new PreBalance(drivetrain)
    //         .andThen(
    //             new InstantCommand(
    //                 () -> {
    //                   enableXstance();
    //                 })));
    // TAB_MAIN.add("Pre-Balance", new PreBalance(drivetrain));
    // TAB_MAIN.addString(
    //     "Height",
    //     () -> {
    //       return this.getHeight().toString();
    //     });
    // TAB_MATCH.addString(
    //     "Height",
    //     () -> {
    //       return this.getHeight().toString();
    //     });
    TAB_MATCH.addBoolean("Double Substation", () -> doubleSubstation);
    TAB_MATCH.addBoolean("Switch Status", () -> wrist.hasHitHardstop());
    TAB_MATCH.add(
        "FRONT CUBE COLLECT",
        new CollectSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_FLOOR));
    // TAB_COMMANDS.add("Extend to 12in", new ExtendArm(arm, 12));
    // TAB_COMMANDS.add("Rotate to 45deg", new RotateArm(arm, 45));
    // TAB_COMMANDS.add("Flip Wrist to true", new FlipWrist(wrist, true));
    // TAB_COMMANDS.add(
    //     "Auto Score Sequence",
    //     new AutoScoreSequence(
    //         arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_FRONT_CONE_TOP));
    // TAB_COMMANDS.add(
    //     "Auto",
    //     new AutoScoreSequence(
    //         arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_FRONT_CONE_TOP));

    // TAB_ARM.add("Seq 45 pos", new SequentialCommandTest(arm, wrist, 16, 45, 3289));
    // TAB_ARM.add("Seq -45 pos", new SequentialCommandTest(arm, wrist, 16, -45, 3289));
    // armTab.add("Collect Back Cube", );

    // TAB_ARM.add("Go Home", new GoHome(arm, wrist));

    // TAB_COMMANDS.add("Stop Claw", new StopClawRollers(claw));

    // TAB_COMMANDS.add("Scan Battery", new InstantCommand(() -> BatteryTracker.scanBattery(10.0)));

    SendableChooser<PositionConfig> collectionChooser = new SendableChooser<>();
    // collectionChooser.setDefaultOption("TEST_POS", TEST_POS);
    // collectionChooser.addOption("TEST_NEG", TEST_NEG);
    collectionChooser.addOption("BACK_CUBE_FLOOR", BACK_CUBE_FLOOR);
    collectionChooser.addOption("BACK_CONE_FLOOR", BACK_CONE_FLOOR);
    collectionChooser.addOption("BACK_CONE_FLOOR_TIPPED", BACK_CONE_FLOOR_TIPPED);
    collectionChooser.addOption("FRONT_CUBE_FLOOR", FRONT_CUBE_FLOOR);
    collectionChooser.addOption("FRONT_CONE_FLOOR", FRONT_CONE_FLOOR);
    collectionChooser.addOption("FRONT_CONE_FLOOR_TIPPED", FRONT_CONE_FLOOR_TIPPED);
    // collectionChooser.addOption("BACK_CUBE_FLOOR", BACK_CUBE_FLOOR); // score
    // collectionChooser.addOption("BACK_CONE_FLOOR_TIPPED", BACK_CONE_FLOOR_TIPPED);
    // collectionChooser.addOption("BACK_CONE_TOP", BACK_CONE_TOP);
    // collectionChooser.addOption("BACK_CONE_MEDIUM", BACK_CONE_MEDIUM);
    // collectionChooser.addOption("BACK_CUBE_TOP", BACK_CUBE_TOP);
    // collectionChooser.addOption("BACK_CUBE_MEDIUM", BACK_CUBE_MEDIUM);
    collectionChooser.addOption("FRONT_CONE_TOP", FRONT_CONE_TOP);
    collectionChooser.addOption("FRONT_CONE_MEDIUM", FRONT_CONE_MEDIUM);
    collectionChooser.addOption("FRONT_CUBE_MEDIUM", FRONT_CUBE_MEDIUM);
    collectionChooser.addOption("FRONT_CUBE_TOP", FRONT_CUBE_TOP);
    collectionChooser.addOption("FRONT_CONE_TOP", FRONT_CONE_TOP);
    // collectionChooser.addOption("FRONT_SINGLE_SUBSTATION", FRONT_SINGLE_SUBSTATION);
    collectionChooser.addOption("BACK_SINGLE_SUBSTATION", BACK_SINGLE_SUBSTATION);
    // collectionChooser.addOption("FRONT_DOUBLE_SUBSTATION", FRONT_DOUBLE_SUBSTATION);
    collectionChooser.addOption("BACK_DOUBLE_SUBSTATION", BACK_DOUBLE_SUBSTATION);
    collectionChooser.addOption("FRONT_CONE_FLOOR_TIPPED_LONG", FRONT_CONE_FLOOR_TIPPED_LONG);
    collectionChooser.addOption("BACK_CUBE_FLOOR_LONG", BACK_CUBE_FLOOR_LONG);
    TAB_MAIN2.add("Collect Chooser", collectionChooser).withPosition(0, 0);

    SendableChooser<PositionConfig> ScoreChooser = new SendableChooser<>();
    ScoreChooser.addOption("SPIT_BACK_CUBE_FLOOR_LONG", SPIT_BACK_CUBE_FLOOR_LONG); // score
    ScoreChooser.addOption("BACK_CUBE_FLOOR", BACK_CUBE_FLOOR); // score
    ScoreChooser.addOption("BACK_CONE_FLOOR_TIPPED", PositionConfigs.BACK_CONE_FLOOR_TIPPED);
    ScoreChooser.addOption("BACK_CONE_TOP", PositionConfigs.BACK_CONE_TOP);
    ScoreChooser.addOption("BACK_CONE_MEDIUM", PositionConfigs.BACK_CONE_MEDIUM);
    ScoreChooser.addOption("BACK_CUBE_TOP", PositionConfigs.BACK_CUBE_TOP);
    ScoreChooser.addOption("BACK_CUBE_MEDIUM", PositionConfigs.BACK_CUBE_MEDIUM);
    ScoreChooser.addOption("FRONT_CONE_MEDIUM", PositionConfigs.FRONT_CONE_MEDIUM);
    ScoreChooser.addOption("FRONT_CUBE_MEDIUM", PositionConfigs.FRONT_CUBE_MEDIUM);
    ScoreChooser.addOption("FRONT_CUBE_TOP", PositionConfigs.FRONT_CUBE_TOP);
    ScoreChooser.addOption("FRONT_CONE_TOP", PositionConfigs.FRONT_CONE_TOP);
    TAB_MAIN2.add("Score Chooser", ScoreChooser).withPosition(0, 1);

    TAB_MAIN2
        .add("Score Sequence", new ScoreSequence(arm, wrist, claw, ScoreChooser::getSelected))
        .withPosition(1, 1);

    TAB_MAIN2
        .add(
            "Collect Sequence",
            new CollectSequence(arm, wrist, claw, collectionChooser::getSelected))
        .withPosition(1, 0);
    TAB_MAIN2
        .add("Eject Game Piece", new EjectGamePiece(claw).withTimeout(0.25))
        .withPosition(2, 1);
    // TAB_MAIN.add("Angle 0, Length 1", new SetArm(arm, () -> 0, () -> 1, () -> true));
    // TAB_MAIN.add("Angle 25, Length 1", new SetArm(arm, () -> 25, () -> 1, () -> false));
    // TAB_MAIN.add("Angle 45, Length 1", new SetArm(arm, () -> 45, () -> 1, () -> false));
    // TAB_MAIN.add("Angle 65, Length 1", new SetArm(arm, () -> 65, () -> 1, () -> false));
    // TAB_MAIN.add("Angle 90, Length 1", new SetArm(arm, () -> 90, () -> 1, () -> false));
    // TAB_MAIN.add("Angle 45, Length 10", new SetArm(arm, () -> 45, () -> 10, () -> false));
    // TAB_MAIN.add("Angle 45, Length 20", new SetArm(arm, () -> 45, () -> 20, () -> false));
    // TAB_MAIN.add("Angle 45, Length 36", new SetArm(arm, () -> 45, () -> 36, () -> false));
    TAB_MAIN2.add("Set Home", new GoHome(arm, wrist)).withPosition(2, 0);
    TAB_MAIN2.add(
        "SPIT_BACK_CUBE_FLOOR_LONG",
        new SequentialCommandGroup(
            new ScoreSequence(
                    arm, wrist, claw, () -> Constants.PositionConfigs.SPIT_BACK_CUBE_FLOOR_LONG)
                .withTimeout(
                    2.0) // timeout of extention, make longer than expected so that position is good
                .andThen(new EjectGamePiece(claw).withTimeout(.4)) // 2910 auto spit timeout
                .andThen(new SetArm(arm, () -> -90, () -> 1, () -> true))));
    // TAB_MAIN.add("Balance", new Balance(drivetrain));
    // TAB_MAIN.add(
    //     "Closest Cone",
    //     new DriveToPose(
    //         drivetrain,
    //         () -> {
    //           final Pose2d currentPose = drivetrain.getPose();
    //           if (currentPose.getX() > 2.5) {
    //             DriverStation.reportWarning("X too far away, not auto driving to cone node",
    // false);
    //             return drivetrain.getPose();
    //           }
    //           final double currentY = currentPose.getY();
    //           try {
    //             final double closestNodeY =
    //                 Constants.OnTheFly.CONE_NODES_Y.stream()
    //                     .filter(
    //                         (Double y) ->
    //                             Util.isWithinTolerance(
    //                                 currentY, y, Constants.OnTheFly.NODE_Y_TOLERANCE))
    //                     .findFirst()
    //                     .get();
    //             return new Pose2d(
    //                 Constants.OnTheFly.GRID_X, closestNodeY, Rotation2d.fromDegrees(180));
    //           } catch (Exception e) {
    //             DriverStation.reportWarning(
    //                 "Not close enough, not auto driving to cone node", false);
    //             return drivetrain.getPose();
    //           }
    //         }));
  }

  public boolean shouldScore() {
    return true;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    // reset gyro to 0 degrees
    new Trigger(driverController::getBackButton)
        .onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance

    // Creates a new Trigger object for the `Right bumper` button that collects cones
    // driverController.rightBumper().onTrue(new CollectGamePiece(claw, GamePiece.CONE));

    // Creates a new Trigger object for the `left bumper` button that collects cubes
    // driverController.leftBumper().onTrue(new CollectGamePiece(claw, GamePiece.CUBE));

    // Creates a new Trigger object for the `right trigger` button that releases the game piece
    // driverController.rightTrigger().onTrue(new ReleaseGamePiece(claw));

    // Creates a new Trigger object for the `left trigger` buttonthat stops all the rollers
    // driverController.leftTrigger().onTrue(new StopClawRollers(claw));

    // driverController
    //     .y()
    //     .onTrue(
    //         new CollectSequence(arm, wrist, claw, () ->
    // Constants.PositionConfigs.BACK_CUBE_FLOOR));
    // driverController
    //     .rightBumper()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new ParallelCommandGroup(
    //                 new SetArm(arm, () -> -45.0, () -> 6.0), new SetWristPosition(1350, wrist)),
    //             new EjectGamePiece(claw),
    //             new ParallelCommandGroup(
    //                 new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION),
    //                 new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
    //             new InstantCommand(() -> arm.setExtensionNominal(), arm)));
    // driverController.start().onTrue(new ParallelCommandGroup(

    // ));

    // coDriverController.pov(0).onTrue(getAutonomousCommand());

    // oi.getWristPosButton()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               wrist.setRotation(true);
    //             }));
    // oi.getWristNegButton()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               wrist.setRotation(false);
    //             }));
    // oi.getRotateButton().onTrue(new InstantCommand(() -> arm.setArmAngle(45)));
    new Trigger(coDriverController::getRightBumper)
        .onTrue(new InstantCommand(() -> claw.setRollerSpeed(-0.9)))
        .onFalse(new InstantCommand(() -> claw.stopRollers()));
    // .whileTrue(new InstantCommand(() -> claw.setRollerSpeed(-.6)));
    new Trigger(driverController::getRightBumper)
        .onTrue(new CollectSequence(arm, wrist, claw, () -> BACK_CUBE_FLOOR, driverController));

    new Trigger(() -> (driverController.getRightTriggerAxis() > 0.1))
        .whileTrue(
            new TeleopSwerve(
                drivetrain,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX,
                () -> 0.5,
                () -> {
                  return driverController.getPOV();
                },
                driverController::getLeftStickButtonPressed));

    new Trigger(() -> (driverController.getLeftTriggerAxis() > 0.1))
        .whileTrue(
            new TeleopSwerve(
                drivetrain,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX,
                () -> 0.5,
                () -> {
                  return 180;
                },
                driverController::getLeftStickButtonPressed));

    new Trigger(driverController::getLeftBumper)
        .onTrue(
            new SequentialCommandGroup(
                new EjectGamePiece(claw).withTimeout(0.25), new GoHome(arm, wrist)));
    new Trigger(coDriverController::getLeftBumper)
        .onTrue(new EjectGamePiece(claw).withTimeout(.25));
    new Trigger(driverController::getStartButton).onTrue(new GoHome(arm, wrist));
    new Trigger(driverController::getBButton)
        .onTrue(
            new ConditionalCommand(
                new CollectSequence(
                    arm,
                    wrist,
                    claw,
                    () -> Constants.PositionConfigs.FRONT_DOUBLE_SUBSTATION,
                    driverController),
                new CollectSequence(
                    arm,
                    wrist,
                    claw,
                    () -> Constants.PositionConfigs.BACK_SINGLE_SUBSTATION,
                    driverController),
                () -> doubleSubstation == true));

    // new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //         new SetArm(arm, () -> -45.0, () -> 6.0, () -> false), new SetWristPosition(1350,
    // wrist)),
    //     new EjectGamePiece(claw).withTimeout(0.25),
    //     new ParallelCommandGroup(
    //         new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () -> true),
    //         new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
    //     new InstantCommand(() -> arm.setExtensionNominal(), arm)));

    new Trigger(coDriverController::getXButton)
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(
                    () -> {
                      doubleSubstation = true;
                    }),
                new InstantCommand(
                    () -> {
                      doubleSubstation = false;
                    }),
                () -> doubleSubstation == false));
    new Trigger(coDriverController::getAButton)
        .onTrue(new InstantCommand(() -> this.setHeight(Height.FLOOR)));
    new Trigger(coDriverController::getBButton)
        .onTrue(new InstantCommand(() -> this.setHeight(Height.MEDIUM)));
    new Trigger(coDriverController::getYButton)
        .onTrue(new InstantCommand(() -> this.setHeight(Height.HIGH)));

    ConditionalCommand floorScore =
        new ConditionalCommand(
            new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_FLOOR),
            new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_FLOOR),
            () -> claw.isCone());
    ConditionalCommand mediumScore =
        new ConditionalCommand(
            new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_MEDIUM),
            new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_MEDIUM),
            () -> claw.isCone());
    ConditionalCommand highScore =
        new ConditionalCommand(
            new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_TOP),
            new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_TOP),
            () -> claw.isCone());
    new Trigger(driverController::getAButton)
        .onTrue(
            new ConditionalCommand(
                floorScore,
                new ConditionalCommand(mediumScore, highScore, () -> height == Height.MEDIUM),
                () -> height == Height.FLOOR));

    new Trigger(driverController::getYButton)
        .onTrue(
            new CollectSequence(
                arm,
                wrist,
                claw,
                () -> Constants.PositionConfigs.BACK_CONE_FLOOR,
                driverController));
    new Trigger(driverController::getXButton)
        .onTrue(
            new CollectSequence(
                arm,
                wrist,
                claw,
                () -> Constants.PositionConfigs.BACK_CONE_FLOOR_TIPPED,
                driverController));

    // driver

    //     .getTempTopScore()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_TOP),
    //             new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_TOP),
    //             () -> claw.getGamePiece() == GamePiece.CONE));

    // driver
    //     .getTempMedScore()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_MEDIUM),
    //             new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_MEDIUM),
    //             () -> claw.getGamePiece() == GamePiece.CONE));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    autoEventMap.put("event1", Commands.print("passed marker 1"));
    autoEventMap.put("event2", Commands.print("passed marker 2"));

    // build auto path commands
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "Straight (shop)", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command autoTest =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(0), drivetrain, true),
                auto1Paths.get(0).getMarkers(),
                autoEventMap),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // build auto path commands
    List<PathPlannerTrajectory> auto3Paths =
        PathPlanner.loadPathGroup(
            "3 Piece", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command auto3Piece = Commands.sequence(new FollowPath(auto3Paths.get(0), drivetrain, true));

    // add commands to the auto chooser
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);
    // autoChooser.addOption("3 Piece", auto3Piece);

    // "auto" command for tuning the drive velocity PID
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0, false), drivetrain))));

    // "auto" command for characterizing the drivetrain
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));

    final HashMap<String, Command> someEventMap = new HashMap<>();
    someEventMap.put(
        "Score Cone",
        new AutoScoreSequence(arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP));
    someEventMap.put("Collect Cube", new CollectSequence(arm, wrist, claw, () -> BACK_CUBE_FLOOR));
    someEventMap.put(
        "Score Cube",
        new AutoScoreSequence(
            arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_FRONT_CUBE_TOP));
    // TODO this auto does not fully work reliably
    autoChooser.addOption(
        "New Auto",
        AutoPathHelper.followPath(drivetrain, "Some Auto", someEventMap, 3.25, 2.5)
            .andThen(new Balance(drivetrain))
            .andThen(() -> drivetrain.setXStance(), drivetrain));

    final HashMap<String, Command> TwoPieceNoCableEventMap = new HashMap<>();
    TwoPieceNoCableEventMap.put(
        "Go Home",
        new ParallelCommandGroup(
            new SetArm(arm, () -> 0, () -> 1, () -> true),
            new SetWristPosition(ANGLE_STRAIGHT, wrist)));
    TwoPieceNoCableEventMap.put(
        "Collect Cube",
        new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)
            .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist).withTimeout(2)));
    TwoPieceNoCableEventMap.put(
        "Score Cube Prep", new SetArm(arm, () -> -49.5, () -> 1, () -> true));

    TwoPieceNoCableEventMap.put(
        "Score Cube",
        new AutoScoreSequence(
            arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_FRONT_CUBE_TOP));
    TwoPieceNoCableEventMap.put("Go Home 2", new GoHome(arm, wrist).withTimeout(2));
    TwoPieceNoCableEventMap.put("Auto Balance", new Balance(drivetrain));

    // TODO Change this back from prebalance

    final HashMap<String, Command> ThreePieceNoCableEventMap = new HashMap<>();
    ThreePieceNoCableEventMap.putAll(TwoPieceNoCableEventMap);

    ThreePieceNoCableEventMap.put(
        "Collect Cube Long",
        new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR_LONG)
            .withTimeout(2.5)
            .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist).withTimeout(2)));

    ThreePieceNoCableEventMap.put(
        "Score Cube Medium",
        new AutoScoreSequence(arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_MEDIUM)
            // .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist)));

    ThreePieceNoCableEventMap.put(
        "Score Cube Medium Extended",
        new AutoScoreSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_MEDIUM_EXTENDED)
            // .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist)));

    ThreePieceNoCableEventMap.put(
        "Score Cube Almost Low",
        new AutoScoreSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_ALMOST_FLOOR_CUBE)
            // .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist)));

    ThreePieceNoCableEventMap.put(
        "Score Cube Back Floor Long",
        new ScoreSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.SPIT_BACK_CUBE_FLOOR_LONG)
            // .withTimeout(
            //     2.0) // timeout of extention, make longer than expected so that position is good
            .andThen(new EjectGamePiece(claw).withTimeout(.4)) // 2910 auto spit timeout
            .andThen(new SetArm(arm, () -> -90, () -> 1, () -> true)));

    ThreePieceNoCableEventMap.put(
        "Score Cube Prep Medium", new SetArm(arm, () -> -49.5, () -> 1, () -> true));

    ThreePieceNoCableEventMap.put(
        "Score Cube Prep Low", new SetArm(arm, () -> -90, () -> 8, () -> false));

    ThreePieceNoCableEventMap.put(
        "Collect Cube with timeout",
        new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)
            .withTimeout(1.6)
            .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist).withTimeout(2)));

    final HashMap<String, Command> ThreePieceBalanceEventMap = new HashMap<>();
    ThreePieceBalanceEventMap.putAll(ThreePieceNoCableEventMap);
    ThreePieceBalanceEventMap.put("Balance", new PreBalance(drivetrain));

    autoChooser.addOption(
        "TwoPieceBalanceCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO)
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "TwoPieceBalanceCable", TwoPieceNoCableEventMap, 3.25, 2.5)));

    autoChooser.addOption(
        "ThreePieceNoCable",
        // new InstantCommand(() -> setShouldUseVision(true)).addThen
        new InstantCommand(() -> claw.setCone(), claw)
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(new GoHome(arm, wrist).withTimeout(1.0))
            .andThen(new InstantCommand(() -> arm.setExtensionNominal()))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "ThreePieceNoCable", ThreePieceNoCableEventMap, 2.75, 2.5))
            .andThen(new EjectGamePiece(claw).withTimeout(0.3))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "ThreePieceCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO)
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "ThreePieceCable", ThreePieceNoCableEventMap, 3.5, 2.25)));

    autoChooser.addOption(
        "Barker3Piece",
        new InstantCommand(() -> setShouldUseVision(true))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            // .andThen(new GoHome(arm, wrist))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "Barker3Piece", ThreePieceNoCableEventMap, 3.25, 2.5))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "Barker3PieceTwisty",
        new InstantCommand(() -> setShouldUseVision(true))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            // .andThen(new GoHome(arm, wrist))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "Barker3PieceTwisty", ThreePieceNoCableEventMap, 3.25, 2.35))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "2910 Red",
        new InstantCommand(() -> setShouldUseVision(true))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            // .andThen(new GoHome(arm, wrist).withTimeout(1.0))
            // .andThen(new InstantCommand(() -> arm.setExtensionNominal()))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "2910 Red", ThreePieceNoCableEventMap, 3.25, 2.75))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "2910 Blue",
        new InstantCommand(() -> setShouldUseVision(true))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            // .andThen(new GoHome(arm, wrist).withTimeout(1.0))
            // .andThen(new InstantCommand(() -> arm.setExtensionNominal()))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "2910 Blue", ThreePieceNoCableEventMap, 3.25, 2.75))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "TwoPieceBalanceNoCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO)
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "TwoPieceNoCable", TwoPieceNoCableEventMap, 3.25, 2.5)));

    autoChooser.addOption(
        "ThreePieceBalance",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO)
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "ThreePieceBalance", ThreePieceBalanceEventMap, 3.75, 2.7))
            .andThen(new PreBalance(drivetrain)));

    autoChooser.addOption(
        "CubeMobility",
        new WaitCommand(0)
            // TODO set with timeout value if needed. MAX of 2 seconds. In middle of field at 7-8
            // seconds.
            .andThen(new CollectGamePiece(claw, GamePiece.CUBE).withTimeout(0.2))
            .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_TOP))
            .andThen(new GoHome(arm, wrist))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "MobilityCube", ThreePieceBalanceEventMap, 1.5, 1))
            .andThen(new Balance(drivetrain)));

    autoChooser.addOption(
        "MobilityCone",
        new WaitCommand(0)
            // TODO set with timeout value if needed. MAX of 2 seconds. In middle of field at 7-8
            // seconds.
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(new GoHome(arm, wrist))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "MobilityCone", ThreePieceBalanceEventMap, 1.5, 1))
            .andThen(new Balance(drivetrain)));

    autoChooser.addOption(
        "MobilityConeCable",
        new WaitCommand(0)
            // TODO set with timeout value if needed. MAX of 2 seconds. In middle of field at 7-8
            // seconds.
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(new GoHome(arm, wrist))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "MobilityConeCable", ThreePieceBalanceEventMap, 1.5, 1))
            .andThen(new Balance(drivetrain)));
    // autoChooser.addOption(

    //     "Some Auto",
    //     new SequentialCommandGroup(
    //             new ParallelCommandGroup(
    //                 new SetArm(
    //                     arm,
    //                     () -> PositionConfigs.FRONT_CONE_MEDIUM.armRotation,
    //                     () -> PositionConfigs.FRONT_CONE_MEDIUM.armLength,
    //                     () -> false),
    //                 new SetWristPosition(PositionConfigs.FRONT_CONE_MEDIUM.wristRotation,
    // wrist)),
    //             new EjectGamePiece(claw).withTimeout(0.25),
    //             new ParallelCommandGroup(
    //                 new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () ->
    // true),
    //                 new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
    //             new InstantCommand(() -> arm.setExtensionNominal(), arm))
    //         .andThen(
    //             AutoPathHelper.followPath(
    //                 drivetrain, "Some Auto", someEventMap)) // add auto paths here
    //         .andThen(new Balance(drivetrain))
    //         .andThen(() -> drivetrain.setXStance(), drivetrain));

    // armTab.addDouble("desired angle", () -> 0);
    // armTab.addDouble("desired length", () -> 0);
    // TAB_ARM.add("SA-Home", new SyncedArm(arm, () -> 0.1, () -> 0.1));
    // TAB_ARM.add(
    //     "SA-HighCone",
    //     new SyncedArm(
    //         arm,
    //         () -> PositionConfigs.FRONT_CONE_TOP.armRotation,
    //         () -> PositionConfigs.FRONT_CONE_TOP.armLength));
    // TAB_ARM.add(
    //     "SA-MidCone",
    //     new SyncedArm(
    //         arm,
    //         () -> PositionConfigs.FRONT_CONE_MEDIUM.armRotation,
    //         () -> PositionConfigs.FRONT_CONE_MEDIUM.armLength));
    // TAB_ARM.add(
    //     "SA-CollectBack",
    //     new SyncedArm(
    //         arm,
    //         () -> PositionConfigs.BACK_CONE_FLOOR.armRotation,
    //         () -> PositionConfigs.BACK_CONE_FLOOR.armLength));
    TAB_MATCH.add(autoChooser);

    TAB_MATCH.add("Shoot Cube", new ShootCube(claw));
    TAB_MATCH.add("Re-Home Wrist", new HomeWrist(wrist));
    TAB_MATCH.addBoolean("Cone", () -> claw.isCone());
    TAB_MATCH.addBoolean("Cube", () -> claw.isCube());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setHeight(Height height) {
    this.height = height;
  }

  public Height getHeight() {
    return height;
  }

  public void enableFieldRelative() {
    drivetrain.enableFieldRelative();
  }

  public void enableXstance() {
    drivetrain.enableXstance();
  }

  public void disableXstance() {
    drivetrain.disableXstance();
  }

  public void setShouldUseVision(boolean shouldUse) {
    shouldUseVision = shouldUse;
  }
}
