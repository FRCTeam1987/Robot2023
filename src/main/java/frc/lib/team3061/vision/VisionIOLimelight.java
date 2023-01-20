package frc.lib.team3061.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import org.jetbrains.annotations.NotNull;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

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

    public Pose3d getBestBotPose() {
        for (LimeLightInstance limeLight : limeLights) {
            if (limeLight.getSeenTags() > 1) {
                return limeLight.getBotPose();
            }
            return limeLight.getBotPose();
        }
        return null;
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
        LIMELIGHT_TAB.addNumber(limelightNameShort + " Tag Num", this::getAprilTag);
        //LIMELIGHT_TAB.addBoolean(limelightNameShort + " Target Visible", this::canSeeTarget);
        //LIMELIGHT_TAB.addNumber(limelightNameShort + " Targets Seen", this::getSeenTags);
        //LIMELIGHT_TAB.addString(limelightNameShort + " pose", this::getBotPoseStr);
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
    public JSONObject getJsonObject() {
        try {
            return (JSONObject) new JSONParser().parse(inst.getTable(limelightName).getEntry("json").getString(null));
        } catch (ParseException ignored) {
            return null;
        }
    }
    public int getSeenTags() {
        try {
            return ((JSONArray) ((JSONObject) getJsonObject().get("Results")).get("Fiducial")).size();
        } catch (Exception ignored) {
            return 0;
        }
    }
    public double getLatencyMillis() {
        return inst.getTable(limelightName).getEntry("tl").getDouble(-1.0);
    }
    public Pose3d getBotPose() {
        try {
            double[] pose = inst.getTable(limelightName).getEntry("botpose").getDoubleArray(new double[]{0,0,0,0,0,0});
            return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
        } catch (Exception e) {
            return null;
        }
    }
    public String getBotPoseStr() {
        try {
            return getBotPose().toString();
        } catch (Exception e) {
            return "No pose detected";
        }
    }
    public double getXAxis() {
        return inst.getTable(limelightName).getEntry("tx").getDouble(0);
    }
    public double getAprilTag() {
        return inst.getTable(limelightName).getEntry("tid").getDouble(-1.0);
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

