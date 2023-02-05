package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;

import java.util.concurrent.atomic.AtomicLong;


public class CollectGamePiece extends SequentialCommandGroup {
    private static final double collectPercent = 0.75;
    private static final double collectAmps = 35;
    private static final double secondsToContinueCollecting = 4.0;

    private static final int delayCurrentCheckMillis = 600;

    public CollectGamePiece(Claw claw) {
        AtomicLong time = new AtomicLong();
        addCommands(
                new StartEndCommand(
                        () -> {
                            claw.setRollerSpeed(collectPercent);
                            time.set(System.currentTimeMillis());
                        },
                        () -> {
                            new WaitCommand(secondsToContinueCollecting);
                            claw.stallRollers();
                        },
                        claw)
                        .until(() -> (claw.getCurrent() > collectAmps && System.currentTimeMillis() > time.get() + delayCurrentCheckMillis)));
    }
}