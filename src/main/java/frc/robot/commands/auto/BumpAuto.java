// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.Limelight.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.commands.AutoScoreSequence;
import frc.robot.commands.CollectSequence;
import frc.robot.commands.GoHome;
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
public class BumpAuto extends SequentialCommandGroup {

  public final double MAX_VELOCITY = 3.25;
  public final double MAX_ACCELERATION = 2.5;

  public final HashMap<String, Command> eventMap01 = new HashMap<>();
  public final HashMap<String, Command> eventMap02 = new HashMap<>();

  /** Creates a new BumpAuto. */
  public BumpAuto(final Arm arm, final Claw claw, final Drivetrain drive, final Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    eventMap02.put("CubeScorePrep", new SetArm(arm, () -> -49.5, () -> 1, () -> true));

    addCommands(
        new InstantCommand(() -> claw.setCone(), claw),
        new AutoScoreSequenceNoHome(
            arm, wrist, claw, () -> Constants.PositionConfigs.FRONT_CONE_TOP_AUTO),
        new GoHome(arm, wrist).withTimeout(0.5),
        AutoPathHelper.followPath(drive, "BumpAuto01", eventMap01, MAX_VELOCITY,
        MAX_ACCELERATION),
        new ParallelRaceGroup(
            new CollectSequence(arm, wrist, claw, () -> Constants.PositionConfigs.BACK_CONE_FLOOR)
                .andThen(new InstantCommand(() -> claw.setGamePiece(GamePiece.CUBE))),
            new DriveToPiece(drive, () -> -0.5, GamePiece.CONE)),
        new GoHome(arm, wrist).withTimeout(2),
        AutoPathHelper.followPath(drive, "BumpAuto02", eventMap02, MAX_VELOCITY,
        MAX_ACCELERATION),
        new DriveToScore(drive, claw),
        new AutoScoreSequence(
            arm, wrist, claw, () -> Constants.PositionConfigs.AUTO_FRONT_CUBE_TOP),
        new GoHome(arm, wrist).withTimeout(0.5)
        );
  }
}
