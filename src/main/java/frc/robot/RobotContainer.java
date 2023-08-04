// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.PositionConfigs.*;
import static frc.robot.subsystems.wrist.Wrist.ANGLE_STRAIGHT;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.robot.Constants.PositionConfig;
import frc.robot.Constants.PositionConfigs;
import frc.robot.commands.*;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.auto.AutoPathHelper;
import frc.robot.commands.auto.AutoScoreSequenceNoHome;
import frc.robot.commands.auto.Balance;
import frc.robot.commands.auto.PreBalance;
import frc.robot.commands.wrist.HomeWrist;
import frc.robot.configs.CompRobotConfig;
import frc.robot.subsystems.MultiLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.claw.ClawIOSparkMAX;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOTalonSRX;
import java.util.HashMap;

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
  private boolean shouldUseVision = false;

  private boolean doubleSubstation = false;

  public enum Height {
    HIGH,
    MEDIUM,
    FLOOR,
    NONE
  }

  private Arm arm;
  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // RobotContainer singleton
  private static final RobotContainer robotContainer = new RobotContainer();
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
    config = new CompRobotConfig();

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

    arm =
        new Arm(
            new ArmIOTalonFX(
                config.getArmLeaderMotorID(),
                config.getArmFollowerMotorID(),
                config.getArmCanCoderID(),
                config.getArmTelescopeID(),
                config.getCANBusName()));

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            () -> 1,
            () -> driverController.getPOV(),
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
    TAB_MATCH.addBoolean("Double Substation", () -> doubleSubstation);
    TAB_MATCH.addBoolean("Switch Status", () -> wrist.hasHitHardstop());
    TAB_MATCH.add(
        "FRONT CUBE COLLECT",
        new CollectSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_FLOOR));

    SendableChooser<PositionConfig> collectionChooser = new SendableChooser<>();
    collectionChooser.addOption("BACK_CONE_FLOOR", BACK_CONE_FLOOR);
    collectionChooser.addOption("BACK_CONE_FLOOR_TIPPED", BACK_CONE_FLOOR_TIPPED);
    collectionChooser.addOption("BACK_CUBE_FLOOR", BACK_CUBE_FLOOR);
    collectionChooser.addOption("BACK_CUBE_FLOOR_LONG", BACK_CUBE_FLOOR_LONG);
    collectionChooser.addOption("BACK_DOUBLE_SUBSTATION", BACK_DOUBLE_SUBSTATION);
    collectionChooser.addOption("BACK_SINGLE_SUBSTATION", BACK_SINGLE_SUBSTATION);
    collectionChooser.addOption("FRONT_CONE_FLOOR", FRONT_CONE_FLOOR);
    collectionChooser.addOption("FRONT_CONE_FLOOR_TIPPED", FRONT_CONE_FLOOR_TIPPED);
    collectionChooser.addOption("FRONT_CONE_FLOOR_TIPPED_LONG", FRONT_CONE_FLOOR_TIPPED_LONG);
    collectionChooser.addOption("FRONT_CONE_MEDIUM", FRONT_CONE_MEDIUM);
    collectionChooser.addOption("FRONT_CONE_TOP", FRONT_CONE_TOP);
    collectionChooser.addOption("FRONT_CUBE_FLOOR", FRONT_CUBE_FLOOR);
    collectionChooser.addOption("FRONT_CUBE_MEDIUM", FRONT_CUBE_MEDIUM);
    collectionChooser.addOption("FRONT_CUBE_TOP", FRONT_CUBE_TOP);

    TAB_MAIN.add("Collect Chooser", collectionChooser).withPosition(0, 0);

    SendableChooser<PositionConfig> scoreChooser = new SendableChooser<>();
    scoreChooser.addOption("BACK_CONE_FLOOR_TIPPED", PositionConfigs.BACK_CONE_FLOOR_TIPPED);
    scoreChooser.addOption("BACK_CONE_MEDIUM", PositionConfigs.BACK_CONE_MEDIUM);
    scoreChooser.addOption("BACK_CONE_TOP", PositionConfigs.BACK_CONE_TOP);
    scoreChooser.addOption("BACK_CUBE_FLOOR", BACK_CUBE_FLOOR);
    scoreChooser.addOption("BACK_CUBE_MEDIUM", PositionConfigs.BACK_CUBE_MEDIUM);
    scoreChooser.addOption("BACK_CUBE_TOP", PositionConfigs.BACK_CUBE_TOP);
    scoreChooser.addOption("FRONT_CONE_MEDIUM", PositionConfigs.FRONT_CONE_MEDIUM);
    scoreChooser.addOption("FRONT_CONE_TOP", PositionConfigs.FRONT_CONE_TOP);
    scoreChooser.addOption("FRONT_CUBE_MEDIUM", PositionConfigs.FRONT_CUBE_MEDIUM);
    scoreChooser.addOption("FRONT_CUBE_TOP", PositionConfigs.FRONT_CUBE_TOP);
    scoreChooser.addOption("SPIT_BACK_CUBE_FLOOR_LONG", SPIT_BACK_CUBE_FLOOR_LONG);

    TAB_MAIN.add("Score Chooser", scoreChooser).withPosition(0, 1);

    TAB_MAIN
        .add("Score Sequence", new ScoreSequence(arm, wrist, scoreChooser::getSelected))
        .withPosition(1, 1);

    TAB_MAIN
        .add(
            "Collect Sequence",
            new CollectSequence(arm, wrist, claw, collectionChooser::getSelected))
        .withPosition(1, 0);

    TAB_MAIN.add("Eject Game Piece", new EjectGamePiece(claw).withTimeout(0.25)).withPosition(2, 1);

    TAB_MAIN.add("Set Home", new GoHome(arm, wrist)).withPosition(2, 0);
    TAB_MAIN.add(
        "SPIT_BACK_CUBE_FLOOR_LONG",
        new SequentialCommandGroup(
            new ScoreSequence(arm, wrist, () -> Constants.PositionConfigs.SPIT_BACK_CUBE_FLOOR_LONG)
                .withTimeout(
                    2.0) // timeout of extension, make longer than expected so that position is good
                .andThen(new EjectGamePiece(claw).withTimeout(.4)) // 2910 auto spit timeout
                .andThen(new SetArm(arm, () -> -90, () -> 1, () -> true))));
  }

  public boolean shouldScore() {
    return true;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    // reset gyro to 0 degrees
    new Trigger(driverController::getBackButton)
        .onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    new Trigger(coDriverController::getRightBumper)
        .onTrue(new InstantCommand(() -> claw.setRollerSpeed(-0.9)))
        .onFalse(new InstantCommand(() -> claw.stopRollers()));
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
                () -> driverController.getPOV(),
                driverController::getLeftStickButtonPressed));

    new Trigger(() -> (driverController.getLeftTriggerAxis() > 0.1))
        .whileTrue(
            new TeleopSwerve(
                drivetrain,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX,
                () -> 0.5,
                () -> 180,
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
                () -> doubleSubstation));

    new Trigger(coDriverController::getXButton)
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> doubleSubstation = true),
                new InstantCommand(() -> doubleSubstation = false),
                () -> !doubleSubstation));
    new Trigger(coDriverController::getAButton)
        .onTrue(new InstantCommand(() -> this.setHeight(Height.FLOOR)));
    new Trigger(coDriverController::getBButton)
        .onTrue(new InstantCommand(() -> this.setHeight(Height.MEDIUM)));
    new Trigger(coDriverController::getYButton)
        .onTrue(new InstantCommand(() -> this.setHeight(Height.HIGH)));

    ConditionalCommand floorScore =
        new ConditionalCommand(
            new ScoreSequence(arm, wrist, () -> PositionConfigs.FRONT_CONE_FLOOR),
            new ScoreSequence(arm, wrist, () -> PositionConfigs.FRONT_CUBE_FLOOR),
            () -> claw.isCone());
    ConditionalCommand mediumScore =
        new ConditionalCommand(
            new ScoreSequence(arm, wrist, () -> PositionConfigs.FRONT_CONE_MEDIUM),
            new ScoreSequence(arm, wrist, () -> PositionConfigs.FRONT_CUBE_MEDIUM),
            () -> claw.isCone());
    ConditionalCommand highScore =
        new ConditionalCommand(
            new ScoreSequence(arm, wrist, () -> PositionConfigs.FRONT_CONE_TOP),
            new ScoreSequence(arm, wrist, () -> PositionConfigs.FRONT_CUBE_TOP),
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
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

    final HashMap<String, Command> autoEventMap = new HashMap<>();

    autoEventMap.put("Auto Balance", new Balance(drivetrain));

    autoEventMap.put("Balance", new PreBalance(drivetrain));

    autoEventMap.put(
        "Collect Cube",
        new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)
            .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist).withTimeout(2)));

    autoEventMap.put(
        "Collect Cube Long",
        new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR_LONG)
            .withTimeout(2.5)
            .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist).withTimeout(2)));

    autoEventMap.put(
        "Collect Cube with timeout",
        new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)
            .withTimeout(1.6)
            .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(new GoHome(arm, wrist).withTimeout(2)));

    autoEventMap.put(
        "Go Home",
        new ParallelCommandGroup(
            new SetArm(arm, () -> 0, () -> 1, () -> true),
            new SetWristPosition(ANGLE_STRAIGHT, wrist)));

    autoEventMap.put("Go Home 2", new GoHome(arm, wrist).withTimeout(2));

    autoEventMap.put(
        "Score Cube",
        new AutoScoreSequence(
            arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_FRONT_CUBE_TOP));

    autoEventMap.put(
        "Score Cube Almost Low",
        new AutoScoreSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_ALMOST_FLOOR_CUBE)
            .andThen(new GoHome(arm, wrist)));

    autoEventMap.put(
        "Score Cube Back Floor Long",
        new ScoreSequence(arm, wrist, () -> Constants.PositionConfigs.SPIT_BACK_CUBE_FLOOR_LONG)
            .andThen(new EjectGamePiece(claw).withTimeout(.4)) // 2910 auto spit timeout
            .andThen(new SetArm(arm, () -> -90, () -> 1, () -> true)));

    autoEventMap.put(
        "Score Cube Medium",
        new AutoScoreSequence(arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_MEDIUM)
            .andThen(new GoHome(arm, wrist)));

    autoEventMap.put(
        "Score Cube Medium Extended",
        new AutoScoreSequence(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_MEDIUM_EXTENDED)
            .andThen(new GoHome(arm, wrist)));

    autoEventMap.put("Score Cube Prep", new SetArm(arm, () -> -49.5, () -> 1, () -> true));

    autoEventMap.put("Score Cube Prep Low", new SetArm(arm, () -> -90, () -> 8, () -> false));

    autoEventMap.put("Score Cube Prep Medium", new SetArm(arm, () -> -49.5, () -> 1, () -> true));

    autoChooser.addOption(
        "TwoPieceBalanceCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO)
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "TwoPieceBalanceCable", autoEventMap, 3.25, 2.5)));

    autoChooser.addOption(
        "ThreePieceNoCable",
        new InstantCommand(() -> claw.setCone(), claw)
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(new GoHome(arm, wrist).withTimeout(1.0))
            .andThen(new InstantCommand(() -> arm.setExtensionNominal()))
            .andThen(
                AutoPathHelper.followPath(drivetrain, "ThreePieceNoCable", autoEventMap, 2.75, 2.5))
            .andThen(new EjectGamePiece(claw).withTimeout(0.3))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "ThreePieceCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO)
            .andThen(
                AutoPathHelper.followPath(drivetrain, "ThreePieceCable", autoEventMap, 3.5, 2.25)));

    autoChooser.addOption(
        "Barker3PieceTwisty",
        new InstantCommand(() -> setShouldUseVision(true))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(
                AutoPathHelper.followPath(
                    drivetrain, "Barker3PieceTwisty", autoEventMap, 3.25, 2.35))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "2910 Red",
        (new InstantCommand(() -> setShouldUseVision(true)))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(AutoPathHelper.followPath(drivetrain, "2910 Red", autoEventMap, 3.25, 2.75))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "2910 Blue",
        new InstantCommand(() -> setShouldUseVision(true))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(AutoPathHelper.followPath(drivetrain, "2910 Blue", autoEventMap, 3.25, 2.75))
            .andThen(new GoHome(arm, wrist)));

    autoChooser.addOption(
        "TwoPieceBalanceNoCable",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO)
            .andThen(new GoHome(arm, wrist).withTimeout(1))
            .andThen(
                AutoPathHelper.followPath(drivetrain, "TwoPieceNoCable", autoEventMap, 3.25, 2.5))
            .andThen(new Balance(drivetrain)));

    autoChooser.addOption(
        "ThreePieceBalance",
        new AutoScoreSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO)
            .andThen(
                AutoPathHelper.followPath(drivetrain, "ThreePieceBalance", autoEventMap, 3.75, 2.7))
            .andThen(new PreBalance(drivetrain)));

    autoChooser.addOption(
        "CubeMobility",
        new WaitCommand(0) // Max of 2 seconds
            .andThen(new CollectGamePiece(claw, GamePiece.CUBE).withTimeout(0.2))
            .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_TOP))
            .andThen(new GoHome(arm, wrist))
            .andThen(AutoPathHelper.followPath(drivetrain, "MobilityCube", autoEventMap, 1.5, 1))
            .andThen(new Balance(drivetrain)));

    autoChooser.addOption(
        "MobilityCone",
        new WaitCommand(0) // Max of 2 seconds
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(new GoHome(arm, wrist))
            .andThen(AutoPathHelper.followPath(drivetrain, "MobilityCone", autoEventMap, 1.5, 1))
            .andThen(new Balance(drivetrain)));

    autoChooser.addOption(
        "MobilityConeCable",
        new WaitCommand(0) // Max of 2 seconds
            .andThen(
                new AutoScoreSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO))
            .andThen(new GoHome(arm, wrist))
            .andThen(
                AutoPathHelper.followPath(drivetrain, "MobilityConeCable", autoEventMap, 1.5, 1))
            .andThen(new Balance(drivetrain)));

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
