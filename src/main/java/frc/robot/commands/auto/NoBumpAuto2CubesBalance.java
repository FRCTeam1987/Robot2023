// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.CollectSequenceNoHome;
import frc.robot.commands.GoHome;
import frc.robot.commands.SetWristPosition;
import frc.robot.commands.arm.SetArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.wrist.Wrist;
import java.util.HashMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoBumpAuto2CubesBalance extends SequentialCommandGroup {

  public final double MAX_V = 3.5;
  public final double MAX_A = 3.25;

  public final HashMap<String, Command> eventMap01 = new HashMap<>();
  public final HashMap<String, Command> eventMap02 = new HashMap<>();
  public final HashMap<String, Command> eventMap03 = new HashMap<>();
  public final HashMap<String, Command> eventMap04 = new HashMap<>();

  private double startTime = 0;

  /** Creates a new NoBumpAuto2Cubes. */
  public NoBumpAuto2CubesBalance(
      final Arm arm, final Claw claw, final Drivetrain drive, final Wrist wrist) {

    eventMap01.put("GoHome", new GoHome(arm, wrist));

    eventMap02.put("GoHome", new GoHome(arm, wrist));
    eventMap02.put(
        "CubeScorePrep",
        new ParallelCommandGroup(
            new SetArm(arm, () -> -49.5, () -> 4, () -> true),
            new SetWristPosition(2045 + Constants.INSTALLED_ARM.getWristOffset(), wrist)));
    eventMap04.put(
        "CubeScorePrep",
        new ParallelCommandGroup(
            new SetArm(arm, () -> -65.4, () -> 6, () -> true),
            new SetWristPosition(2045 + Constants.INSTALLED_ARM.getWristOffset(), wrist)));
    eventMap03.put("GoHome", new GoHome(arm, wrist));
    eventMap04.put("GoHome", new GoHome(arm, wrist));

    addCommands(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        // Step 1: Score preload
        new InstantCommand(() -> claw.setCone(), claw),
        // new WaitCommand(0.26),
        new AutoScoreSequenceNoHomeWait(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO),
        // Step 2: Drive almost to the first game piece
        new ParallelCommandGroup(
            AutoPathHelper.followPath(drive, "NoBumpAuto01", eventMap01, MAX_V, MAX_A),
            new InstantCommand(() -> RobotContainer.setCubePipeline())),
        // Step 3: Collect the first game piece
        new ParallelRaceGroup(
            new DriveToPiece(drive, () -> -2.25, GamePiece.CUBE),
            new CollectSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)),
        new WaitCommand(0.06)
            .andThen(
                () -> {
                  RobotContainer.setAprilTagPipeline();
                  claw.setGamePiece(GamePiece.CUBE);
                }),
        // Step 4: Drive almost to the scoring location
        AutoPathHelper.followPathNoReset(drive, "NoBumpAuto02", eventMap02, MAX_V, MAX_A),
        // Step 5: Score the first game piece
        new DriveToScore(drive, claw).withTimeout(1.0),
        new AutoScoreSequenceNoHome(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_TOP_AUTO),
        // Step 6: Drive almost to the second game piece
        new ParallelCommandGroup(
            AutoPathHelper.followPathNoRotationReset(
                drive, "NoBumpAuto03", eventMap03, MAX_V, MAX_A),
            new InstantCommand(() -> RobotContainer.setConePipeline())), // Shouldnt need
        // Step 7: Collect the second game piece
        new ParallelRaceGroup(
                new DriveToPiece(drive, () -> -1.5, GamePiece.CONE),
                new CollectSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CONE_FLOOR))
            .withTimeout(2.0),
        new WaitCommand(0.04),
        // Step 8: Drive almost to the scoring location
        new ParallelCommandGroup(
            new InstantCommand(() -> RobotContainer.setAprilTagPipeline()), // Shouldnt need
            new InstantCommand(
                () -> {
                  claw.setGamePiece(GamePiece.CONE);
                }),
            AutoPathHelper.followPathNoReset(
                drive, "NoBumpAuto04Balance", eventMap04, MAX_V, MAX_A)),
        // Step 10: Finish
        new InstantCommand(
            () -> SmartDashboard.putNumber("Auto Time", Timer.getFPGATimestamp() - startTime)),
        new InstantCommand(() -> RobotContainer.setAprilTagPipeline()));
  }
}
