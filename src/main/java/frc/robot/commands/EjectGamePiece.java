package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;

import java.util.concurrent.atomic.AtomicLong;

public class EjectGamePiece extends SequentialCommandGroup {

    public EjectGamePiece(Claw claw) {
        addCommands(
                new StartEndCommand(
                        () -> {
                            claw.setRollerSpeed(SmartDashboard.getNumber("ejectSpeed", -1.0));
                            System.out.println("start");
                        },
                        () -> {
                            new WaitCommand(4.5);
                            claw.stopRollers();
                            System.out.println("End");
                        },
                        claw)
                        .until(() -> (claw.getCurrent() < SmartDashboard.getNumber("ejectAmps", 15))));
    }
}