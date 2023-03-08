package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class ParallelCommandTest extends ParallelCommandGroup {
  public ParallelCommandTest(
      final Arm ARM, final Wrist WRIST, final int INCHES, final double ANGLE, final int ticks) {
    addRequirements(ARM, WRIST);
    addCommands(new RotateAndExtendArm(ARM, ANGLE, INCHES), new SetWristPosition(ticks, WRIST));
  }
}
