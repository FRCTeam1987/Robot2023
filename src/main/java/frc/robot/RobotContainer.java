// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.PositionConfigs.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PositionConfig;
import frc.robot.Constants.PositionConfigs;
import frc.robot.commands.*;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.arm.SyncedArm;
import frc.robot.commands.auto.AutoPathHelper;
import frc.robot.commands.auto.Balance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
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
  private DrivetrainSubsystem drivetrain;
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
  XboxController driverController = new XboxController(1); // Creates a CommandXboxController on port 1.
  XboxController coDriverController = new XboxController(2);

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */

    // create real, simulated, or replay subsystems based on the mode and robot specified

    SwerveModule frontLeftModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                TAB_DRIVETRAIN
                    .getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.CAN_BUS_NAME)
            .withSteerMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.CAN_BUS_NAME)
            .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.CAN_BUS_NAME)
            .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
            .withModuleId(2)
            .build();

    SwerveModule frontRightModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                TAB_DRIVETRAIN
                    .getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.CAN_BUS_NAME)
            .withSteerMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.CAN_BUS_NAME)
            .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.CAN_BUS_NAME)
            .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
            .withModuleId(1)
            .build();

    SwerveModule backLeftModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                TAB_DRIVETRAIN
                    .getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.CAN_BUS_NAME)
            .withSteerMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.CAN_BUS_NAME)
            .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.CAN_BUS_NAME)
            .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
            .withModuleId(3)
            .build();

    SwerveModule backRightModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                TAB_DRIVETRAIN
                    .getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.CAN_BUS_NAME)
            .withSteerMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.CAN_BUS_NAME)
            .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.CAN_BUS_NAME)
            .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
            .withModuleId(4)
            .build();

    drivetrain =
        new DrivetrainSubsystem(
            Constants.GYRO_ID, frontLeftModule, frontRightModule, backLeftModule, backRightModule);
    drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, driverController::getLeftX, driverController::getLeftY, driverController::getRightX));
    // new Pneumatics(new PneumaticsIORev()); // Needs CTRE for practice bot
    wrist = new Wrist(Constants.WRIST_ROTATOR_MOTOR);
    claw = new Claw(Constants.CLAW_MOTOR_ID);
    claw.setDefaultCommand(new DefaultClawRollersSpin(claw));
    // temp
    vision = new Vision("limelight-fl", "limelight-bl", "limelight-br"); // "limelight-fr"
    arm =
        new Arm(
            Constants.ARM_LEADER_MOTOR,
            Constants.ARM_FOLLOWER_MOTOR,
            Constants.ARM_CANCODER,
            Constants.ARM_TELESCOPE_MOTOR,
            Constants.ARM_POTENTIOMETER_ANALOG_ID,
            Constants.CAN_BUS_NAME);

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

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
    // new Trigger(driverController::getBackButton).toggleOnTrue(
    //     Commands.either(
    //         Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
    //         Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
    //         drivetrain::getFieldRelative));


    // reset gyro to 0 degrees
    new Trigger(driverController::getBackButton).onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    // new Trigger(driverController::getBackButton).onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    // new Trigger(driverController::getBackButton).onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

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
    new Trigger(driverController::getRightBumper)
        .onTrue(new CollectSequence(arm, wrist, claw, () -> BACK_CUBE_FLOOR));
        new Trigger(driverController::getLeftBumper)
        .onTrue(
            new SequentialCommandGroup(
                new EjectGamePiece(claw).withTimeout(0.25), new GoHome(arm, wrist)));
        new Trigger(driverController::getStartButton).onTrue(new GoHome(arm, wrist));
    // new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //         new SetArm(arm, () -> -45.0, () -> 6.0, () -> false), new SetWristPosition(1350,
    // wrist)),
    //     new EjectGamePiece(claw).withTimeout(0.25),
    //     new ParallelCommandGroup(
    //         new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () -> true),
    //         new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
    //     new InstantCommand(() -> arm.setExtensionNominal(), arm)));

    new Trigger(coDriverController::getAButton).onTrue(new InstantCommand(() -> System.out.println("Floor")));
    new Trigger(coDriverController::getXButton).onTrue(new InstantCommand(() -> this.setHeight(Height.MEDIUM)));
    new Trigger(coDriverController::getYButton).onTrue(new InstantCommand(() -> this.setHeight(Height.HIGH)));

    new Trigger(driverController::getBButton)
        .onTrue(new SetArmHeight(getHeight(), arm, wrist, claw, this.driverController));

    new Trigger(driverController::getYButton)
        .onTrue(
            new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CONE_FLOOR));
    new Trigger(driverController::getXButton)
        .onTrue(
            new CollectSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CONE_FLOOR_TIPPED));

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
            "testPaths1",
            Constants.AUTO_MAX_SPEED_METERS_PER_SECOND,
            Constants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    // Command autoTest =
    //     Commands.sequence(
    //         new FollowPathWithEvents(
    //             new FollowPath(auto1Paths.get(0), drivetrain, true),
    //             auto1Paths.get(0).getMarkers(),
    //             autoEventMap),
    //         // Commands.runOnce(drivetrain::enableXstance, drivetrain),
    //         Commands.waitSeconds(5.0)
    //         // Commands.runOnce(drivetrain::disableXstance, drivetrain)
    //         );

    // build auto path commands
    List<PathPlannerTrajectory> auto3Paths =
        PathPlanner.loadPathGroup(
            "3 Piece",
            Constants.AUTO_MAX_SPEED_METERS_PER_SECOND,
            Constants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //Command auto3Piece = Commands.sequence(new FollowPath(auto3Paths.get(0), drivetrain, true));

    // add commands to the auto chooser
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    //autoChooser.addOption("Test Path", autoTest);
    // autoChooser.addOption("3 Piece", auto3Piece);

    // "auto" command for tuning the drive velocity PID
    // autoChooser.addOption(
    //     "Drive Velocity Tuning",
    //     Commands.sequence(
    //         Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
    //         Commands.deadline(
    //             Commands.waitSeconds(5.0),
    //             Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0), drivetrain))));

    // "auto" command for characterizing the drivetrain
    // autoChooser.addOption(
    //     "Drive Characterization",
    //     new FeedForwardCharacterization(
    //         drivetrain,
    //         true,
    //         new FeedForwardCharacterizationData("drive"),
    //         drivetrain::runCharacterizationVolts,
    //         drivetrain::getCharacterizationVelocity));

    final HashMap<String, Command> someEventMap = new HashMap<>();
    someEventMap.put(
        "Score Cone",
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
    // TODO this auto does not fully work reliably
    // autoChooser.addOption(
    //     "New Auto",
    //     AutoPathHelper.followPath(drivetrain, "Some Auto", someEventMap)
    //         .andThen(new Balance(drivetrain))
    //         //.andThen(() -> drivetrain.setXStance(), drivetrain));
    //         ;
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
