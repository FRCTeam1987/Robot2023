package frc.lib.team3061.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

import java.util.ArrayList;

public class VisionIOLimelight implements VisionIO {
    private Alert noCameraConnectedAlert;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    ArrayList<LimeLightInstance> limeLights = new ArrayList<>();

    public VisionIOLimelight(String... names) {
        for (String name : names) {
            LimeLightInstance limeLightInstance = new LimeLightInstance(name);
            limeLights.add(limeLightInstance);
            noCameraConnectedAlert = new Alert("limelight not connected (" + name + ")", AlertType.WARNING);
        }
    }
    @Override
    public synchronized void updateInputs(VisionIO.VisionIOInputs inputs) {
        noCameraConnectedAlert.set(!inst.isConnected());
    }
}

class LimeLightInstance {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("Limelights");
    String limelightName;

    public LimeLightInstance(String name) {
        this.limelightName = name;

        String limelightNameShort = limelightName.replace("limelight-", "");

        inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
        inst.getTable(limelightName).getEntry("pipeline").setNumber(4);

        LIMELIGHT_TAB.addNumber(limelightNameShort + " X Axis ", this::getXAxis);
        LIMELIGHT_TAB.addNumber(limelightNameShort + " Y Axis", this::getYAxis);
        LIMELIGHT_TAB.addBoolean(limelightNameShort + " Target Visible", this::canSeeTarget);

        InstantCommand ledsOnCommand = new InstantCommand(this::turnOnLEDs);
        ledsOnCommand.runsWhenDisabled();
        LIMELIGHT_TAB.add(limelightNameShort + " LEDs ON", ledsOnCommand);

        InstantCommand ledsOffCommand = new InstantCommand(this::turnOffLEDs);
        ledsOffCommand.runsWhenDisabled();
        LIMELIGHT_TAB.add(limelightNameShort + " LEDs OFF", ledsOffCommand);
    }

    public double getYAxis() {
        return inst.getTable(limelightName).getEntry("ty").getDouble(0);
    }

    public double getXAxis() {
        return inst.getTable(limelightName).getEntry("tx").getDouble(0);
    }

    public boolean canSeeTarget() {
        return inst.getTable(limelightName).getEntry("tv").getDouble(0) == 1;
    }

    public void turnOnLEDs() {
        inst.getTable(limelightName).getEntry("ledMode").setNumber(3);
    }

    public void turnOffLEDs() {
        inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
    }

}
