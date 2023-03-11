// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.auto.AutoPathHelper;
import frc.robot.commands.auto.Balance;
import frc.robot.configs.CompRobotConfig;
import frc.robot.configs.TestRobotConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.claw.Claw;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};

  private RobotConfig config;
  private Drivetrain drivetrain;
  private Wrist wrist;
  private Claw claw;

  private Vision vision;
  private Arm arm;
  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

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
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

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
        new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

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
    // 0);
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm Tab");

    SmartDashboard.putData("Extend Arm to 12 Inches", new ExtendArm(arm, 12));
    SmartDashboard.putData("Rotate Arm to 45 Degrees", new RotateArm(arm, 45));
    SmartDashboard.putData("Flip Wrist to true", new FlipWrist(wrist, true));

    armTab.add("Sequential Command 45 pos", new SequentialCommandTest(arm, wrist, 16, 45, 3289));
    armTab.add("Sequential Command -45 pos", new SequentialCommandTest(arm, wrist, 16, -45, 3289));
    // armTab.add("Collect Back Cube", );

    armTab.add("Go Home", new GoHome(arm, wrist));

    SmartDashboard.putData("Stop Claw", new StopClawRollers(claw));

    SmartDashboard.putData(
        "Scan Battery", new InstantCommand(() -> BatteryTracker.scanBattery(10.0)));

    SendableChooser<PositionConfig> collectionChooser = new SendableChooser<>();
    collectionChooser.setDefaultOption("TEST POS", PositionConfigs.TEST_POS);
    collectionChooser.addOption("TEST NEG", PositionConfigs.TEST_NEG);
    collectionChooser.addOption("back cube", PositionConfigs.BACK_CUBE_FLOOR);
    collectionChooser.addOption("back cone", PositionConfigs.BACK_CONE_FLOOR);
    collectionChooser.addOption("back cone tipped", PositionConfigs.BACK_CONE_FLOOR_TIPPED);
    collectionChooser.addOption("front cube", PositionConfigs.FRONT_CUBE_FLOOR);
    collectionChooser.addOption("front cone", PositionConfigs.FRONT_CONE_FLOOR);
    collectionChooser.addOption("front cone tipped", PositionConfigs.FRONT_CONE_FLOOR_TIPPED);
    collectionChooser.addOption("BACK_CUBE_FLOOR", PositionConfigs.BACK_CUBE_FLOOR); // score
    collectionChooser.addOption("BACK_CONE_FLOOR_TIPPED", PositionConfigs.BACK_CONE_FLOOR_TIPPED);
    collectionChooser.addOption("BACK_CONE_TOP", PositionConfigs.BACK_CONE_TOP);
    collectionChooser.addOption("BACK_CONE_MEDIUM", PositionConfigs.BACK_CONE_MEDIUM);
    collectionChooser.addOption("BACK_CUBE_TOP", PositionConfigs.BACK_CUBE_TOP);
    collectionChooser.addOption("BACK_CUBE_MEDIUM", PositionConfigs.BACK_CUBE_MEDIUM);
    collectionChooser.addOption("FRONT_CONE_TOP", PositionConfigs.FRONT_CONE_TOP);
    collectionChooser.addOption("FRONT_CONE_MEDIUM", PositionConfigs.FRONT_CONE_MEDIUM);
    collectionChooser.addOption("FRONT_CUBE_MEDIUM", PositionConfigs.FRONT_CUBE_MEDIUM);
    collectionChooser.addOption("FRONT_CUBE_TOP", PositionConfigs.FRONT_CUBE_TOP);
    collectionChooser.addOption("FRONT_CONE_TOP", PositionConfigs.FRONT_CONE_TOP);
    collectionChooser.addOption("FRONT_SINGLE_SUBSTATION", PositionConfigs.FRONT_SINGLE_SUBSTATION);
    collectionChooser.addOption("FRONT_DOUBLE_SUBSTATION", PositionConfigs.FRONT_DOUBLE_SUBSTATION);
    collectionChooser.addOption(
        "front cone tipped long", PositionConfigs.FRONT_CONE_FLOOR_TIPPED_LONG);
    Shuffleboard.getTab("MAIN").add("Collect Chooser", collectionChooser);

    SendableChooser<PositionConfig> ScoreChooser = new SendableChooser<>();
    ScoreChooser.addOption("BACK_CUBE_FLOOR", PositionConfigs.BACK_CUBE_FLOOR); // score
    ScoreChooser.addOption("BACK_CONE_FLOOR_TIPPED", PositionConfigs.BACK_CONE_FLOOR_TIPPED);
    ScoreChooser.addOption("BACK_CONE_TOP", PositionConfigs.BACK_CONE_TOP);
    ScoreChooser.addOption("BACK_CONE_MEDIUM", PositionConfigs.BACK_CONE_MEDIUM);
    ScoreChooser.addOption("BACK_CUBE_TOP", PositionConfigs.BACK_CUBE_TOP);
    ScoreChooser.addOption("BACK_CUBE_MEDIUM", PositionConfigs.BACK_CUBE_MEDIUM);
    ScoreChooser.addOption("FRONT_CONE_MEDIUM", PositionConfigs.FRONT_CONE_MEDIUM);
    ScoreChooser.addOption("FRONT_CUBE_MEDIUM", PositionConfigs.FRONT_CUBE_MEDIUM);
    ScoreChooser.addOption("FRONT_CUBE_TOP", PositionConfigs.FRONT_CUBE_TOP);
    ScoreChooser.addOption("FRONT_CONE_TOP", PositionConfigs.FRONT_CONE_TOP);
    Shuffleboard.getTab("MAIN").add("Score Chooser", ScoreChooser);

    Shuffleboard.getTab("MAIN")
        .add("Score Sequence", new ScoreSequence(arm, wrist, claw, ScoreChooser::getSelected));

    Shuffleboard.getTab("MAIN")
        .add(
            "Collect Sequence",
            new CollectSequence(arm, wrist, claw, collectionChooser::getSelected));
    Shuffleboard.getTab("MAIN").add("Eject Game Piece", new EjectGamePiece(claw).withTimeout(0.25));
    Shuffleboard.getTab("MAIN")
        .add("angle 25, length 1", new SetArm(arm, () -> 25, () -> 1, () -> false));
    Shuffleboard.getTab("MAIN")
        .add("angle 45, length 1", new SetArm(arm, () -> 45, () -> 1, () -> false));
    Shuffleboard.getTab("MAIN")
        .add("angle 65, length 1", new SetArm(arm, () -> 65, () -> 1, () -> false));
    Shuffleboard.getTab("MAIN")
        .add("angle 90, length 1", new SetArm(arm, () -> 90, () -> 1, () -> false));
    Shuffleboard.getTab("MAIN")
        .add("angle 45, length 10", new SetArm(arm, () -> 45, () -> 10, () -> false));
    Shuffleboard.getTab("MAIN")
        .add("angle 45, length 20", new SetArm(arm, () -> 45, () -> 20, () -> false));
    Shuffleboard.getTab("MAIN")
        .add("angle 45, length 36", new SetArm(arm, () -> 45, () -> 36, () -> false));
    Shuffleboard.getTab("MAIN").add("set home", new GoHome(arm, wrist));
    Shuffleboard.getTab("MAIN").add("Balance", new Balance(drivetrain));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle

    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

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
    oi.getTempCollectCube()
        .onTrue(
            new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR));
    oi.getTempScore()
        .onTrue(
            new SequentialCommandGroup(
                new EjectGamePiece(claw).withTimeout(0.25), new GoHome(arm, wrist)));
    oi.getTempGoHome().onTrue(new GoHome(arm, wrist));
    // new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //         new SetArm(arm, () -> -45.0, () -> 6.0, () -> false), new SetWristPosition(1350,
    // wrist)),
    //     new EjectGamePiece(claw).withTimeout(0.25),
    //     new ParallelCommandGroup(
    //         new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () -> true),
    //         new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
    //     new InstantCommand(() -> arm.setExtensionNominal(), arm)));

    oi.getTempCollectCone()
        .onTrue(
            new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CONE_FLOOR));
    oi.getTempCollectConeGround()
        .onTrue(
            new CollectSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CONE_FLOOR_TIPPED));
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
    Command auto3Piece =
        Commands.sequence(
            new FollowPath(auto3Paths.get(0), drivetrain, true));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);
    autoChooser.addOption("3 Piece", auto3Piece);

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
    someEventMap.put("Score Cone", new WaitCommand(2));
    someEventMap.put(
        "Collect Cube",
        new CollectSequence(arm, wrist, claw, () -> PositionConfigs.BACK_CUBE_FLOOR));
    someEventMap.put(
        "Score Cube",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArm(arm, () -> -45.0, () -> 6.0, () -> false),
                new SetWristPosition(1350, wrist)),
            new EjectGamePiece(claw).withTimeout(0.25),
            new ParallelCommandGroup(
                new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () -> true),
                new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
            new InstantCommand(() -> arm.setExtensionNominal(), arm)));
    // TODO this auto does not fully work reliably
    autoChooser.addOption(
        "Some Auto",
        new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetArm(
                        arm,
                        () -> PositionConfigs.FRONT_CONE_MEDIUM.armRotation,
                        () -> PositionConfigs.FRONT_CONE_MEDIUM.armLength,
                        () -> false),
                    new SetWristPosition(PositionConfigs.FRONT_CONE_MEDIUM.wristRotation, wrist)),
                new EjectGamePiece(claw).withTimeout(0.25),
                new ParallelCommandGroup(
                    new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () -> true),
                    new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
                new InstantCommand(() -> arm.setExtensionNominal(), arm))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "Some Auto", someEventMap)) // add auto paths here
            .andThen(new Balance(drivetrain))
            .andThen(() -> drivetrain.setXStance(), drivetrain));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
