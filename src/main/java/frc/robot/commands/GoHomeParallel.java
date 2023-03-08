package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class GoHomeParallel extends ParallelCommandGroup {
  /** Creates a new GoHome. */
  public GoHomeParallel(final Arm ARM, final Wrist WRIST) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetWristPosition(2289, WRIST), new ExtendArm(ARM, 1), new RotateArm(ARM, 0));
  }
}
