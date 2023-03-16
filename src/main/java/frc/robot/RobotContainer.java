// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.PositionConfigs.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.commands.arm.SyncedArm;
import frc.robot.commands.auto.AutoPathHelper;
import frc.robot.commands.auto.Balance;
import frc.robot.configs.CompRobotConfig;
import frc.robot.configs.TestRobotConfig;
import frc.robot.operator_interface.CoDriver.CoDriverOISelector;
import frc.robot.operator_interface.CoDriver.CoDriverOperatorInterface;
import frc.robot.operator_interface.Driver.DriverOISelector;
import frc.robot.operator_interface.Driver.DriverOperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.claw.ClawIOSparkMAX;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOTalonSRX;
import frc.robot.util.BatteryTracker;
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
  private DriverOperatorInterface driver = new DriverOperatorInterface() {};
  private CoDriverOperatorInterface coDriver = new CoDriverOperatorInterface() {};

  private RobotConfig config;
  private Drivetrain drivetrain;
  private Wrist wrist;
  private Claw claw;
  private Height height = Height.HIGH;

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
  CommandXboxController driverController =
      new CommandXboxController(1); // Creates a CommandXboxController on port 1.
  CommandXboxController coDriverController = new CommandXboxController(2);

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
            // temp
            vision =
                new Vision(
                    new VisionIOLimelight(
                        "limelight-fl", "limelight-bl", "limelight-br")); // "limelight-fr"
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

    updateOI();

    configureAutoCommands();
    configureSmartDashboard();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all the buttons to commands.
   */
  public void updateOI() {
    if (!DriverOISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    driver = DriverOISelector.findOperatorInterface();
    coDriver = CoDriverOISelector.findOperatorInterface();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y-axis specifies the velocity in the x direction
     * and the left joystick's x-axis specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain, driver::getTranslateX, driver::getTranslateY, driver::getRotate));

    configureButtonBindings();
  }

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
    // debugTab.add("Wrist Rotation (Degrees)", wrist.getDegrees()).withSize(2, 2).withPosition(4,

    TAB_COMMANDS.add("Extend to 12in", new ExtendArm(arm, 12));
    TAB_COMMANDS.add("Rotate to 45deg", new RotateArm(arm, 45));
    TAB_COMMANDS.add("Flip Wrist to true", new FlipWrist(wrist, true));
    TAB_COMMANDS.add(
        "Test Score", new SetArmHeight(getHeight(), arm, wrist, claw, this.driverController));

    TAB_ARM.add("Seq 45 pos", new SequentialCommandTest(arm, wrist, 16, 45, 3289));
    TAB_ARM.add("Seq -45 pos", new SequentialCommandTest(arm, wrist, 16, -45, 3289));
    // armTab.add("Collect Back Cube", );

    TAB_ARM.add("Go Home", new GoHome(arm, wrist));

    TAB_COMMANDS.add("Stop Claw", new StopClawRollers(claw));

    TAB_COMMANDS.add("Scan Battery", new InstantCommand(() -> BatteryTracker.scanBattery(10.0)));

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
    // collectionChooser.addOption("FRONT_CONE_FLOOR_TIPPED_LONG", FRONT_CONE_FLOOR_TIPPED_LONG);
    TAB_MAIN.add("Collect Chooser", collectionChooser);

    SendableChooser<PositionConfig> ScoreChooser = new SendableChooser<>();
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
    TAB_MAIN.add("Score Chooser", ScoreChooser);

    TAB_MAIN.add("Score Sequence", new ScoreSequence(arm, wrist, claw, ScoreChooser::getSelected));

    TAB_MAIN.add(
        "Collect Sequence", new CollectSequence(arm, wrist, claw, collectionChooser::getSelected));
    TAB_MAIN.add("Eject Game Piece", new EjectGamePiece(claw).withTimeout(0.25));
    TAB_MAIN.add("Angle 25, Length 1", new SetArm(arm, () -> 25, () -> 1, () -> false));
    TAB_MAIN.add("Angle 45, Length 1", new SetArm(arm, () -> 45, () -> 1, () -> false));
    TAB_MAIN.add("Angle 65, Length 1", new SetArm(arm, () -> 65, () -> 1, () -> false));
    TAB_MAIN.add("Angle 90, Length 1", new SetArm(arm, () -> 90, () -> 1, () -> false));
    TAB_MAIN.add("Angle 45, Length 10", new SetArm(arm, () -> 45, () -> 10, () -> false));
    TAB_MAIN.add("Angle 45, Length 20", new SetArm(arm, () -> 45, () -> 20, () -> false));
    TAB_MAIN.add("Angle 45, Length 36", new SetArm(arm, () -> 45, () -> 36, () -> false));
    TAB_MAIN.add("Set Home", new GoHome(arm, wrist));
    TAB_MAIN.add("Balance", new Balance(drivetrain));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle
    driver
        .getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    driver.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    driver.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    driver.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

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
    driver
        .getTempCollectCube()
        .onTrue(new CollectSequence(arm, wrist, claw, () -> BACK_CUBE_FLOOR));
    driver
        .getTempEject()
        .onTrue(
            new SequentialCommandGroup(
                new EjectGamePiece(claw).withTimeout(0.25), new GoHome(arm, wrist)));
<<<<<<< Updated upstream
    driver.getTempGoHome().onTrue(new GoHome(arm, wrist));
=======
    new Trigger(coDriverController::getLeftBumper).onTrue(new DumpGamePiece(wrist, claw));
    new Trigger(driverController::getStartButton).onTrue(new GoHome(arm, wrist));
    new Trigger(driverController::getBButton)
        .onTrue(
            new CollectSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.BACK_SINGLE_SUBSTATION));

>>>>>>> Stashed changes
    // new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //         new SetArm(arm, () -> -45.0, () -> 6.0, () -> false), new SetWristPosition(1350,
    // wrist)),
    //     new EjectGamePiece(claw).withTimeout(0.25),
    //     new ParallelCommandGroup(
    //         new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () -> true),
    //         new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
    //     new InstantCommand(() -> arm.setExtensionNominal(), arm)));

    coDriver.getTempFloorScore().onTrue(new InstantCommand(() -> System.out.println("Floor")));
    coDriver.getTempMediumScore().onTrue(new InstantCommand(() -> this.setHeight(Height.MEDIUM)));
    coDriver.getTempHighScore().onTrue(new InstantCommand(() -> this.setHeight(Height.HIGH)));

    driver
        .getTempScore()
        .onTrue(new SetArmHeight(getHeight(), arm, wrist, claw, this.driverController));

    driver
        .getTempCollectConeFloor()
        .onTrue(
            new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CONE_FLOOR));
    driver
        .getTempCollectConeTip()
        .onTrue(
            new CollectSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CONE_FLOOR_TIPPED));

    driver
        .getTempTopScore()
        .onTrue(
            new ConditionalCommand(
                new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_TOP),
                new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_TOP),
                () -> claw.getGamePiece() == GamePiece.CONE));

    driver
        .getTempMedScore()
        .onTrue(
            new ConditionalCommand(
                new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_MEDIUM),
                new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_MEDIUM),
                () -> claw.getGamePiece() == GamePiece.CONE));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    autoEventMap.put("event1", Commands.print("passed marker 1"));
    autoEventMap.put("event2", Commands.print("passed marker 2"));

    // build auto path commands
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "testPaths1", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
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
<<<<<<< Updated upstream
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArm(
                    arm,
                    () -> PositionConfigs.FRONT_CONE_TOP.armRotation,
                    () -> PositionConfigs.FRONT_CONE_TOP.armLength,
                    () -> false),
                new SetWristPosition(PositionConfigs.FRONT_CONE_TOP.wristRotation, wrist)),
            new EjectGamePiece(claw).withTimeout(0.25),
            new GoHome(arm, wrist)));
    someEventMap.put("Collect Cube", new CollectSequence(arm, wrist, claw, () -> BACK_CUBE_FLOOR));
    someEventMap.put(
        "Score Cube",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArm(
                    arm,
                    () -> PositionConfigs.FRONT_CONE_TOP.armRotation,
                    () -> PositionConfigs.FRONT_CONE_TOP.armLength,
                    () -> false),
                new SetWristPosition(PositionConfigs.FRONT_CONE_TOP.wristRotation, wrist)),
            new EjectGamePiece(claw).withTimeout(0.25),
            new ParallelCommandGroup(
                new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () -> true),
                new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
            new InstantCommand(() -> arm.setExtensionNominal(), arm)));
=======
        new AutoScoreSequence(arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP));
    someEventMap.put("Collect Cube", new CollectSequence(arm, wrist, claw, () -> BACK_CUBE_FLOOR));
    someEventMap.put(
        "Score Cube",
        new AutoScoreSequence(arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_TOP));
>>>>>>> Stashed changes
    // TODO this auto does not fully work reliably
    autoChooser.addOption(
        "New Auto",
        AutoPathHelper.followPath(drivetrain, "Some Auto", someEventMap)
            .andThen(new Balance(drivetrain))
            .andThen(() -> drivetrain.setXStance(), drivetrain));
<<<<<<< Updated upstream
=======

    final HashMap<String, Command> TwoPieceNoCableEventMap = new HashMap<>();
    TwoPieceNoCableEventMap.put(
        "Go Home",
        new ParallelCommandGroup(
            new SetArm(arm, () -> 0, () -> 1, () -> true),
            new SetWristPosition(ANGLE_STRAIGHT, wrist)));
    TwoPieceNoCableEventMap.put(
        "Collect Cube",
        new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)
            .andThen(new GoHome(arm, wrist).withTimeout(2)));
    TwoPieceNoCableEventMap.put(
        "Score Cube Prep", new SetArm(arm, () -> -48.5, () -> 5, () -> false));
    TwoPieceNoCableEventMap.put(
        "Score Cube",
        new AutoScoreSequence(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_TOP_AUTO));
    TwoPieceNoCableEventMap.put(
        "Score Cube No Momentum",
        new AutoScoreSequence(arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_TOP));
    TwoPieceNoCableEventMap.put("Go Home 2", new GoHome(arm, wrist).withTimeout(2));
    TwoPieceNoCableEventMap.put("Auto Balance", new Balance(drivetrain));

    final HashMap<String, Command> ThreePieceNoCableEventMap = new HashMap<>();
    ThreePieceNoCableEventMap.putAll(TwoPieceNoCableEventMap);
    ThreePieceNoCableEventMap.put(
        "Score Cube Medium",
        new AutoScoreSequence(arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_MEDIUM));

    autoChooser.addOption(
        "TwoPieceBalanceCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP)
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "TwoPieceBalanceCable", TwoPieceNoCableEventMap)));

    autoChooser.addOption(
        "ThreePieceNoCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP)
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "ThreePieceNoCable", ThreePieceNoCableEventMap)));

    autoChooser.addOption(
        "TwoPieceNoCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP)
            .andThen(
                AutoPathHelper.followPath(drivetrain, "TwoPieceNoCable", TwoPieceNoCableEventMap)));

>>>>>>> Stashed changes
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
    TAB_ARM.add("SA-Home", new SyncedArm(arm, () -> 0.1, () -> 0.1));
    TAB_ARM.add(
        "SA-HighCone",
        new SyncedArm(
            arm,
            () -> PositionConfigs.FRONT_CONE_TOP.armRotation,
            () -> PositionConfigs.FRONT_CONE_TOP.armLength));
    TAB_ARM.add(
        "SA-MidCone",
        new SyncedArm(
            arm,
            () -> PositionConfigs.FRONT_CONE_MEDIUM.armRotation,
            () -> PositionConfigs.FRONT_CONE_MEDIUM.armLength));
    TAB_ARM.add(
        "SA-CollectBack",
        new SyncedArm(
            arm,
            () -> PositionConfigs.BACK_CONE_FLOOR.armRotation,
            () -> PositionConfigs.BACK_CONE_FLOOR.armLength));
    TAB_MAIN.add(autoChooser);
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
}
