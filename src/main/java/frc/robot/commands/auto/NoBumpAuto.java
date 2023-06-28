// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoScoreSequence;
import frc.robot.commands.CollectSequence;
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
public class NoBumpAuto extends SequentialCommandGroup {

  public final double MAX_V = 3.35;
  public final double MAX_A = 3.1;

  public final HashMap<String, Command> eventMap01 = new HashMap<>();
  public final HashMap<String, Command> eventMap02 = new HashMap<>();
  public final HashMap<String, Command> eventMap03 = new HashMap<>();
  public final HashMap<String, Command> eventMap04 = new HashMap<>();

  /** Creates a new NoBumpAuto. */
  public NoBumpAuto(final Arm arm, final Claw claw, final Drivetrain drive, final Wrist wrist) {

    eventMap01.put("GoHome", new GoHome(arm, wrist));

    eventMap02.put("GoHome", new GoHome(arm, wrist));
    eventMap02.put(
        "CubeScorePrep",
        new ParallelCommandGroup(
            new SetArm(arm, () -> -49.5, () -> 4, () -> true),
            new SetWristPosition(2045 + Constants.INSTALLED_ARM.getWristOffset(), wrist)));
    eventMap03.put("GoHome", new GoHome(arm, wrist));
    eventMap04.put("GoHome", new GoHome(arm, wrist));
    eventMap04.put(
        "ConeScorePrep",
        new ParallelCommandGroup(
            new SetArm(arm, () -> -49.5, () -> 4, () -> true),
            new SetWristPosition(2045 + Constants.INSTALLED_ARM.getWristOffset(), wrist)));

    addCommands(
        // Step 1: Score preload
        new InstantCommand(() -> claw.setCone(), claw),
        new AutoScoreSequenceNoHome(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO),
        // Step 2: Drive almost to the first game piece
        AutoPathHelper.followPath(drive, "NoBumpAuto01", eventMap01, MAX_V, MAX_A),
        // Step 3: Collect the first game piece
        new ParallelRaceGroup(
            new DriveToPiece(drive, () -> -2.25, GamePiece.CUBE),
            new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)
                .andThen(new InstantCommand(() -> {
                  claw.setGamePiece(GamePiece.CUBE);
                }))),
        new WaitCommand(0.1).andThen(() -> RobotContainer.setAprilTagPipeline()),
        // Step 4: Drive almost to the scoring location
        AutoPathHelper.followPathNoRotationReset(drive, "NoBumpAuto02", eventMap02, MAX_V, MAX_A),
        // Step 5: Score the first game piece
        new DriveToScore(drive, claw).withTimeout(2.5),
        new AutoScoreSequenceNoHome(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_TOP),
        // Step 6: Drive almost to the second game piece
        AutoPathHelper.followPathNoRotationReset(drive, "NoBumpAuto03", eventMap03, MAX_V, MAX_A),
        // Step 7: Collect the second game piece
        new ParallelRaceGroup(
            new DriveToPiece(drive, () -> -2.0, GamePiece.CONE),
            new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_BACK_CONE_FLOOR)
                .andThen(new InstantCommand(() -> {
                  claw.setGamePiece(GamePiece.CONE);
                }))).andThen(() -> RobotContainer.setRetroReflectPipeline()),
        // Step 8: Drive almost to the scoring location
        AutoPathHelper.followPathNoRotationReset(drive, "NoBumpAuto04", eventMap04, MAX_V, MAX_A),
        // Step 9: Score the second game piece
        new DriveToScore(drive, claw),
        new AutoScoreSequence(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO),
        new InstantCommand(() -> RobotContainer.setAprilTagPipeline()));
  }
}
