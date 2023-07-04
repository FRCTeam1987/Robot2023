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
import frc.robot.commands.AutoScoreSequence;
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
public class BumpAuto2Cubes extends SequentialCommandGroup {

  public final double MAX_VELOCITY = 3.35;
  public final double MAX_ACCELERATION = 3.25;

  public final HashMap<String, Command> eventMap01 = new HashMap<>();
  public final HashMap<String, Command> eventMap02 = new HashMap<>();
  public final HashMap<String, Command> eventMap03 = new HashMap<>();
  public final HashMap<String, Command> eventMap04 = new HashMap<>();

  private double startTime = 0;

  /** Creates a new BumpAuto2Cubes. */
  public BumpAuto2Cubes(final Arm arm, final Claw claw, final Drivetrain drive, final Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    eventMap01.put("GoHome", new GoHome(arm, wrist));
    eventMap02.put(
        "CubeScorePrep",
        new ParallelCommandGroup(
            new SetArm(arm, () -> -49.5, () -> 8, () -> true),
            new SetWristPosition(2045 + Constants.INSTALLED_ARM.getWristOffset(), wrist)));
    eventMap02.put("GoHome", new GoHome(arm, wrist).withTimeout(0.5));
    eventMap03.put("GoHome", new GoHome(arm, wrist));
    eventMap03.put(
        "CubeCollectPrep",
        new ParallelCommandGroup(
            new SetArm(arm, () -> 25, () -> 4, () -> true),
            new SetWristPosition(2731 + Constants.INSTALLED_ARM.getWristOffset(), wrist)));

    eventMap04.put("GoHome", new GoHome(arm, wrist));
    eventMap04.put(
        "CubeScorePrep",
        new ParallelCommandGroup(
            new SetArm(arm, () -> -65.4, () -> 6, () -> true),
            new SetWristPosition(2045 + Constants.INSTALLED_ARM.getWristOffset(), wrist)));

    addCommands(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        new InstantCommand(() -> claw.setCone(), claw),
        new AutoScoreSequenceNoHome(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_MEDIUM),
        new ParallelCommandGroup(
            new InstantCommand(() -> RobotContainer.setCubePipeline()),
            AutoPathHelper.followPath(
                drive, "BumpAuto01", eventMap01, MAX_VELOCITY, MAX_ACCELERATION)),
        new ParallelRaceGroup(
            new DriveToPiece(drive, () -> -2.25, GamePiece.CUBE),
            new CollectSequenceNoHome(
                    arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)
                .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)))),
        // new WaitCommand(0.06),
        new ParallelCommandGroup(
            new InstantCommand(() -> RobotContainer.setAprilTagPipeline()),
            AutoPathHelper.followPathNoRotationReset(
                drive, "BumpAuto02", eventMap02, MAX_VELOCITY, MAX_ACCELERATION)),
        new DriveToScore(drive, claw).withTimeout(2.5),
        new AutoScoreSequenceNoHome(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_TOP),
        new ParallelCommandGroup(
            new InstantCommand(() -> RobotContainer.setCubePipeline()),
            AutoPathHelper.followPathNoRotationReset(
                drive, "BumpAuto03", eventMap03, MAX_VELOCITY, MAX_ACCELERATION)),
        new ParallelRaceGroup(
            new DriveToPiece(drive, () -> -2.0, GamePiece.CUBE),
            new CollectSequenceNoHome(
                arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CUBE_FLOOR)),
        new ParallelCommandGroup(
            new InstantCommand(() -> RobotContainer.setAprilTagPipeline()), // Shouldnt need
            new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE)),
            AutoPathHelper.followPathNoReset(
                drive, "BumpAuto04", eventMap04, MAX_VELOCITY, MAX_ACCELERATION)),
        // Step 9: Score the second game piece
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new SetArm(arm, () -> -65.4, () -> 6, () -> true),
                new SetWristPosition(2045 + Constants.INSTALLED_ARM.getWristOffset(), wrist),
                new WaitCommand(10)),
            new DriveToScore(drive, claw).withTimeout(5)),
        new AutoScoreSequence(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CUBE_MEDIUM_AUTO),
        new InstantCommand(
            () -> SmartDashboard.putNumber("Auto Time", Timer.getFPGATimestamp() - startTime)),
        new InstantCommand(() -> RobotContainer.setAprilTagPipeline()));
  }
}
