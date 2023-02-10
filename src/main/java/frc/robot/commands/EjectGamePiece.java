package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;

public class EjectGamePiece extends SequentialCommandGroup {

  public EjectGamePiece(Claw claw) {
    addCommands(
        new StartEndCommand(
                () -> {
                  claw.setRollerSpeed(-1.0);
                },
                () -> {
                  new WaitCommand(3.5);
                  claw.stopRollers();
                },
                claw)
            .until(() -> (claw.getCurrent() < 15)));
  }
}
