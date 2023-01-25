package frc.lib.team3061.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class VisionIOLimelight implements VisionIO {
    public static VisionIOLimelight getInstance() {
        return instance;
    }
    private static VisionIOLimelight instance;
    public static int row = 0;
    List<VisionIOLimelightBase> limelights = new ArrayList<>();
    public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("Limelights");
    public VisionIOLimelight(String... limelights) {
        instance = this;
        for (String name : limelights) {
            this.limelights.add(new VisionIOLimelightBase(name));
        }
        LIMELIGHT_TAB.addString("Best Bot Pose: ", this::getBestBotPoseStr);
    }
    public VisionIOLimelightBase getIdealLimelight() {
        return Collections.max(limelights, this::compareMaxInt);
    }
    public Pose3d getBestBotPose() {
        return getIdealLimelight().getBotPose();
    }
    //This is for debugging. Remove for production.
    public String getBestBotPoseStr() {
        return getBestBotPose().toString();
    }

    private int compareMaxInt(VisionIOLimelightBase a, VisionIOLimelightBase b) {
        return a.getSeenTags() - a.getSeenTags();
    }


    @Override
    public synchronized void updateInputs(VisionIO.VisionIOInputs inputs) {
        for (VisionIOLimelightBase limelight : limelights) {
            String name = limelight.limelightNameFormatted;
            try {
                inputs.getClass().getField(name + "VisibleTags").setLong(name + "VisibleTags", limelight.getSeenTags());
                inputs.getClass().getField(name + "FrameMillis").setLong(name + "FrameMillis", limelight.getFrameMillis());
                inputs.getClass().getField(name + "Json").set(name + "Json", limelight.getRawJson());
            } catch (IllegalAccessException | NoSuchFieldException e) {
                throw new RuntimeException(e);
            }

        }
    }
}

