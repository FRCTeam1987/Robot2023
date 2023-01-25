package frc.lib.team3061.vision;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Constants.LIMELIGHT_PIPELINE;

public class VisionIOLimelightBase {
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("Limelights");
    public final String limelightName;
    public String limelightNameFormatted = null;

    public VisionIOLimelightBase(String limelightName) {
        int column = 0;
            this.limelightName = limelightName;
            switch(limelightName) {
                case "limelight-fl":
                     limelightNameFormatted = "limelightFrontLeft";
                     break;
                case "limelight-fr":
                    limelightNameFormatted = "limelightFrontRight";
                    break;
                case "limelight-rl":
                    limelightNameFormatted = "limelightRearLeft";
                    break;
                case "limelight-rr":
                    limelightNameFormatted = "limelightRearRight";
                    break;
            }
            String limelightNameShort = limelightName.replace("limelight-", "");

            //inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
            inst.getTable(limelightName).getEntry("pipeline").setNumber(LIMELIGHT_PIPELINE);
            LIMELIGHT_TAB.addNumber(limelightNameShort + " X Axis ", this::getXAxis).withPosition(column++, VisionIOLimelight.row);
            LIMELIGHT_TAB.addNumber(limelightNameShort + " Y Axis", this::getYAxis).withPosition(column++, VisionIOLimelight.row);
            LIMELIGHT_TAB.addNumber(limelightNameShort + " Tag Num", this::getAprilTag).withPosition(column++, VisionIOLimelight.row);
            LIMELIGHT_TAB.addBoolean(limelightNameShort + " Target Visible", this::canSeeTarget).withPosition(column++, VisionIOLimelight.row);
            LIMELIGHT_TAB.addNumber(limelightNameShort + " Targets Seen", this::getSeenTags).withPosition(column++, VisionIOLimelight.row);
            LIMELIGHT_TAB.addString(limelightNameShort + " pose", this::getBotPoseStr).withPosition(column++, VisionIOLimelight.row);
            InstantCommand ledsOnCommand = new InstantCommand(this::turnOnLEDs);
            ledsOnCommand.runsWhenDisabled();
            LIMELIGHT_TAB.add(limelightNameShort + " LEDs ON", ledsOnCommand).withPosition(column++, VisionIOLimelight.row);
            InstantCommand ledsOffCommand = new InstantCommand(this::turnOffLEDs);
            ledsOffCommand.runsWhenDisabled();
            LIMELIGHT_TAB.add(limelightNameShort + " LEDs OFF", ledsOffCommand).withPosition(column++, VisionIOLimelight.row);
            VisionIOLimelight.row++;
    }
    public double getYAxis() {
        return inst.getTable(limelightName).getEntry("ty").getDouble(0);
    }
    public JsonNode getJsonNode() {
        try { return new ObjectMapper().readTree(getRawJson()); } catch (Exception ignored) { }
        return null;
    }
    public String getRawJson() {
        return inst.getTable(limelightName).getEntry("json").getString(null);
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
    public int getSeenTags() {
        try { return getJsonNode().at("/Results/Fiducial").size(); } catch (Exception ignored) { return 0; }
    }
    public long getLatencyMillis() {
        return inst.getTable(limelightName).getEntry("tl").getInteger(-1L);
    }
    public long getFrameMillis() {
        return System.currentTimeMillis() - inst.getTable(limelightName).getEntry("tl").getInteger(-1L);
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
