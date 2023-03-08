package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;

public class RotateAndExtendArm extends CommandBase {

  private final Arm ARM;
  private final double ANGLE;
  private final int INCHES;

  public RotateAndExtendArm(final Arm ARM, final double ANGLE, final int INCHES) {
    this.ARM = ARM;
    this.ANGLE = ANGLE;
    this.INCHES = INCHES;
    addRequirements(this.ARM);
  }

  @Override
  public void initialize() {
    ARM.setArmAngle(ANGLE);
    ARM.setArmLength(INCHES);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      System.out.println("Not interrupted, holding current angle!");
      ARM.holdCurrentAngle(ANGLE);
      // ARM.stallArm();
    }
  }

  @Override
  public boolean isFinished() {
    return (Util.isWithinTolerance(ARM.getArmAngle(), ANGLE, 0.5))
        && (Util.isWithinTolerance(ARM.getArmLength(), INCHES, 1));
  }
}
