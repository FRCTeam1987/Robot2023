package frc.robot.subsystems.wrist;

import static frc.robot.Constants.ADVANTAGE_KIT_ENABLED;
import static frc.robot.Constants.TAB_WRIST;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SetWristPosition;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final DigitalInput wrist_switch = new DigitalInput(0);

  private final double currentThreshold = 10.0; // amps
  public static int ANGLE_STRAIGHT = 1457 + Constants.INSTALLED_ARM.getWristOffset(); // 2062
  public static final int ANGLE_FRONT_MAX = 795; // when telescope extended
  public static final int ANGLE_FRONT_PERPENDICULAR = 447;
  public static final int ANGLE_BACK_PERPENDICULAR = 2439;
  public static final int ANGLE_BACK_MAX = 3393; // when telescope extended
  public static final int ANGLE_BACK_HALF = 2635; // when telescope extended
  public static final int ANGLE_FRONT_HALF = 1924; // when telescope extended

  /** Creates a new Wrist. */
  public Wrist(WristIO io) {
    this.io = io;
    int row = 0;
    TAB_WRIST.add("Set Normal", new InstantCommand(() -> io.setRotation(true))).withSize(2, 1);
    TAB_WRIST.add("Set Invert", new InstantCommand(() -> io.setRotation(false))).withSize(2, 1);

    TAB_WRIST.add("Set Straight", new SetWristPosition(ANGLE_STRAIGHT, this)).withSize(2, 1);
    TAB_WRIST
        .add("Set Front Perpendicular", new SetWristPosition(ANGLE_STRAIGHT + 800, this))
        .withSize(2, 1);
    TAB_WRIST
        .add("Set Back Perpendicular", new SetWristPosition(ANGLE_STRAIGHT - 800, this))
        .withSize(2, 1);
    TAB_WRIST
        .add("Set Back Half Perpendicular", new SetWristPosition(ANGLE_BACK_HALF, this))
        .withSize(2, 1);
    TAB_WRIST
        .add("Set Front Half Perpendicular", new SetWristPosition(ANGLE_FRONT_HALF, this))
        .withSize(2, 1);
    // TAB_WRIST.addBoolean("has hit limit switch", this::hasHitHardstop);
  }

  public double getCurrent() {
    return io.getCurrentAmps();
  }

  public void setRotation(boolean inverted) {
    io.setRotation(inverted);
  }

  public void setPosition(final int ticks) {
    io.setPosition(ticks);
  }

  public int getPosition() {
    return io.getPosition();
  }

  public double getDegrees() {
    return io.getDegrees();
  }

  public void setPercent(final double percent) {
    io.setPercent(percent);
  }

  public void configRelative(final int homeTicks) {
    io.configRelative(homeTicks);
  }

  public boolean hasHitHardstop() {
    return !wrist_switch.get();
  }

  public void periodic() {
    if (ADVANTAGE_KIT_ENABLED) {
      io.updateInputs(inputs);
      Logger.getInstance().processInputs("Wrist", inputs);
    }
  }
}
