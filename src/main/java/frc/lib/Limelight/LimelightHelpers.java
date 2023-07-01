// LimelightHelpers v1.2.1 (March 1, 2023)

package frc.lib.Limelight;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.team3061.util.RobotOdometry;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

public class LimelightHelpers {

  public static class LimelightTargetRetro {

    @JsonProperty("t6cts")
    private double[] cameraPoseTargetSpace;

    @JsonProperty("t6rfs")
    private double[] robotPoseFieldSpace;

    @JsonProperty("t6rts")
    private double[] robotPoseTargetSpace;

    @JsonProperty("t6tcs")
    private double[] targetPoseCameraSpace;

    @JsonProperty("t6trs")
    private double[] targetPoseRobotSpace;

    public Pose3d getCameraPoseTargetSpace() {
      return toPose3D(cameraPoseTargetSpace);
    }

    public Pose3d getRobotPoseFieldSpace() {
      return toPose3D(robotPoseFieldSpace);
    }

    public Pose3d getRobotPoseTargetSpace() {
      return toPose3D(robotPoseTargetSpace);
    }

    public Pose3d getTargetPoseCameraSpace() {
      return toPose3D(targetPoseCameraSpace);
    }

    public Pose3d getTargetPoseRobotSpace() {
      return toPose3D(targetPoseRobotSpace);
    }

    public Pose2d getCameraPoseTargetSpace2D() {
      return toPose2D(cameraPoseTargetSpace);
    }

    public Pose2d getRobotPoseFieldSpace2D() {
      return toPose2D(robotPoseFieldSpace);
    }

    public Pose2d getRobotPoseTargetSpace2D() {
      return toPose2D(robotPoseTargetSpace);
    }

    public Pose2d getTargetPoseCameraSpace2D() {
      return toPose2D(targetPoseCameraSpace);
    }

    public Pose2d getTargetPoseRobotSpace2D() {
      return toPose2D(targetPoseRobotSpace);
    }

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double txPixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double tyPixels;

    @JsonProperty("ts")
    public double ts;

    public LimelightTargetRetro() {
      cameraPoseTargetSpace = new double[6];
      robotPoseFieldSpace = new double[6];
      robotPoseTargetSpace = new double[6];
      targetPoseCameraSpace = new double[6];
      targetPoseRobotSpace = new double[6];
    }
  }

  public static class LimelightTargetFiducial {

    @JsonProperty("fID")
    public double fiducialID;

    @JsonProperty("fam")
    public String fiducialFamily;

    @JsonProperty("t6cts")
    private double[] cameraPoseTargetSpace;

    @JsonProperty("t6rfs")
    private double[] robotPoseFieldSpace;

    @JsonProperty("t6rts")
    private double[] robotPoseTargetSpace;

    @JsonProperty("t6tcs")
    private double[] targetPoseCameraSpace;

    @JsonProperty("t6trs")
    private double[] targetPoseRobotSpace;

    public Pose3d getCameraPoseTargetSpace() {
      return toPose3D(cameraPoseTargetSpace);
    }

    public Pose3d getRobotPoseFieldSpace() {
      return toPose3D(robotPoseFieldSpace);
    }

    public Pose3d getRobotPoseTargetSpace() {
      return toPose3D(robotPoseTargetSpace);
    }

    public Pose3d getTargetPoseCameraSpace() {
      return toPose3D(targetPoseCameraSpace);
    }

    public Pose3d getTargetPoseRobotSpace() {
      return toPose3D(targetPoseRobotSpace);
    }

    public Pose2d getCameraPoseTargetSpace2D() {
      return toPose2D(cameraPoseTargetSpace);
    }

    public Pose2d getRobotPoseFieldSpace2D() {
      return toPose2D(robotPoseFieldSpace);
    }

    public Pose2d getRobotPoseTargetSpace2D() {
      return toPose2D(robotPoseTargetSpace);
    }

    public Pose2d getTargetPoseCameraSpace2D() {
      return toPose2D(targetPoseCameraSpace);
    }

    public Pose2d getTargetPoseRobotSpace2D() {
      return toPose2D(targetPoseRobotSpace);
    }

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double txPixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double tyPixels;

    @JsonProperty("ts")
    public double ts;

    public LimelightTargetFiducial() {
      cameraPoseTargetSpace = new double[6];
      robotPoseFieldSpace = new double[6];
      robotPoseTargetSpace = new double[6];
      targetPoseCameraSpace = new double[6];
      targetPoseRobotSpace = new double[6];
    }
  }

  public static class LimelightTargetBarcode {}

  public static class LimelightTargetClassifier {

    @JsonProperty("class")
    public String className;

    @JsonProperty("classID")
    public double classID;

    @JsonProperty("conf")
    public double confidence;

    @JsonProperty("zone")
    public double zone;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double txPixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double tyPixels;
  }

  public static class LimelightTargetDetector {

    @JsonProperty("class")
    public String className;

    @JsonProperty("classID")
    public double classID;

    @JsonProperty("conf")
    public double confidence;

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double txPixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double tyPixels;
  }

  public static class Results {

    @JsonProperty("pID")
    public double pipelineID;

    @JsonProperty("tl")
    public double latencyPipeline;

    @JsonProperty("cl")
    public double latencyCapture;

    public double latencyjsonParse;

    @JsonProperty("ts")
    public double timestampLIMELIGHTPublish;

    @JsonProperty("tsrio")
    public double timestampRIOFPGACapture;

    @JsonProperty("v")
    @JsonFormat(shape = Shape.NUMBER)
    public boolean valid;

    @JsonProperty("botpose")
    public double[] botpose;

    @JsonProperty("botposewpired")
    public double[] botposeWpired;

    @JsonProperty("botposewpiblue")
    public double[] botposeWpiblue;

    @JsonProperty("t6crs")
    public double[] cameraposeRobotspace;

    public Pose3d getBotPose3d() {
      return toPose3D(botpose);
    }

    public Pose3d getBotPose3dWpiRed() {
      return toPose3D(botposeWpired);
    }

    public Pose3d getBotPose3dWpiBlue() {
      return toPose3D(botposeWpiblue);
    }

    public Pose2d getBotPose2d() {
      return toPose2D(botpose);
    }

    public Pose2d getBotPose2dWpiRed() {
      return toPose2D(botposeWpired);
    }

    public Pose2d getBotPose2dWpiBlue() {
      return toPose2D(botposeWpiblue);
    }

    @JsonProperty("Retro")
    public LimelightTargetRetro[] targetsRetro;

    @JsonProperty("Fiducial")
    public LimelightTargetFiducial[] targetsFiducials;

    @JsonProperty("Classifier")
    public LimelightTargetClassifier[] targetsClassifier;

    @JsonProperty("Detector")
    public LimelightTargetDetector[] targetsDetector;

    @JsonProperty("Barcode")
    public LimelightTargetBarcode[] targetsBarcode;

    public Results() {
      botpose = new double[6];
      botposeWpired = new double[6];
      botposeWpiblue = new double[6];
      cameraposeRobotspace = new double[6];
      targetsRetro = new LimelightTargetRetro[0];
      targetsFiducials = new LimelightTargetFiducial[0];
      targetsClassifier = new LimelightTargetClassifier[0];
      targetsDetector = new LimelightTargetDetector[0];
      targetsBarcode = new LimelightTargetBarcode[0];
    }
  }

  public static class LimelightResults {
    @JsonProperty("Results")
    public Results targetingResults;

    public LimelightResults() {
      targetingResults = new Results();
    }
  }

  private static final String LED_MODE = "ledMode";
  private static final String STREAM = "stream";

  private static ObjectMapper mapper;

  /** Print JSON Parse time to the console in milliseconds */
  static boolean profileJSON = false;

  static final String sanitizeName(String name) {
    if (name.equals("")) {
      return "limelight";
    }
    return name;
  }

  private static Pose3d toPose3D(double[] inData) {
    if (inData.length < 6) {
      DriverStation.reportError("Bad LL 3D Pose Data!", false);
      return new Pose3d();
    }
    return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(
            Units.degreesToRadians(inData[3]),
            Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5])));
  }

  private static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      DriverStation.reportError("Bad LL 2D Pose Data!", false);
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d =
        RobotOdometry.getInstance().getPoseEstimator().getEstimatedPosition().getRotation();
    return new Pose2d(tran2d, r2d);
  }

  public static NetworkTable getLimelightNTTable(
      String tableName) { // Add logs if network table issue persits on enable
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  public static double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
  }

  public static void setLimelightNTDouble(String tableName, String entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).setDouble(val);
  }

  public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
    getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
  }

  public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
  }

  public static String getLimelightNTString(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getString("");
  }

  public static URL getLimelightURLString(String tableName, String request) {
    String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
    URL url;
    try {
      url = new URL(urlString);
      return url;
    } catch (MalformedURLException e) {
      DriverStation.reportError("bad LL URL", false);
    }
    return null;
  }
  /////
  /////

  public static double getTX(String limelightName) {
    return getLimelightNTDouble(limelightName, "tx");
  }

  public static double getTY(String limelightName) {
    return getLimelightNTDouble(limelightName, "ty");
  }

  public static double getTA(String limelightName) {
    return getLimelightNTDouble(limelightName, "ta");
  }

  public static double getLatencyPipeline(String limelightName) {
    return getLimelightNTDouble(limelightName, "tl");
  }

  public static double getLatencyCapture(String limelightName) {
    return getLimelightNTDouble(limelightName, "cl");
  }

  public static double getCurrentPipelineIndex(String limelightName) {
    return getLimelightNTDouble(limelightName, "getpipe");
  }

  public static String getJSONDump(String limelightName) {
    return getLimelightNTString(limelightName, "json");
  }

  public static double[] getBotPose(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose");
  }

  public static double[] getBotPoseWpiRed(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botposewpired");
  }

  public static double[] getBotPoseWpiBlue(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botposewpiblue");
  }

  public static double[] getBotPoseTargetSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botposetargetspace");
  }

  public static double[] getCameraPoseTargetSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "cameraposetargetspace");
  }

  public static double[] getTargetPoseCameraSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "targetposecameraspace");
  }

  public static double[] getTargetPoseRobotSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "targetposerobotspace");
  }

  public static double[] getTargetColor(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "tc");
  }

  public static double getFiducialID(String limelightName) {
    return getLimelightNTDouble(limelightName, "tid");
  }

  public static double getNeuralClassID(String limelightName) {
    return getLimelightNTDouble(limelightName, "tclass");
  }

  /////
  /////

  public static Pose3d getBotPose3d(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
    return toPose3D(poseArray);
  }

  public static Pose3d getBotPose3dwpiRed(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "botposewpired");
    return toPose3D(poseArray);
  }

  public static Pose3d getBotPose3dWpiBlue(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "botposewpiblue");
    return toPose3D(poseArray);
  }

  public static Pose3d getBotPose3dTargetSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "botposetargetspace");
    return toPose3D(poseArray);
  }

  public static Pose3d getCameraPose3dTargetSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "cameraposetargetspace");
    return toPose3D(poseArray);
  }

  public static Pose3d getTargetPose3dCameraSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetposecameraspace");
    return toPose3D(poseArray);
  }

  public static Pose3d getTargetPose3dRobotSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetposerobotspace");
    return toPose3D(poseArray);
  }

  public static Pose3d getCameraPose3dRobotSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "cameraposerobotspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   *
   * @param limelightName
   * @return
   */
  public static Pose2d getBotPose2dwpiBlue(String limelightName) {

    double[] result = getBotPoseWpiBlue(limelightName);
    return toPose2D(result);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   *
   * @param limelightName
   * @return
   */
  public static Pose2d getBotPose2dwpiRed(String limelightName) {

    double[] result = getBotPoseWpiRed(limelightName);
    return toPose2D(result);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   *
   * @param limelightName
   * @return
   */
  public static Pose2d getBotPose2d(String limelightName) {

    double[] result = getBotPose(limelightName);
    return toPose2D(result);
  }

  public static boolean getTV(String limelightName) {
    return 1.0 == getLimelightNTDouble(limelightName, "tv");
  }

  public static void setPipelineIndex(String limelightName, int pipelineIndex) {
    setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
  }

  /** The LEDs will be controlled by Limelight pipeline settings, and not by robot code. */
  public static void setLEDModePipelineControl(String limelightName) {
    setLimelightNTDouble(limelightName, LED_MODE, 0);
  }

  public static void setLEDModeForceOff(String limelightName) {
    setLimelightNTDouble(limelightName, LED_MODE, 1);
  }

  public static void setLEDModeForceBlink(String limelightName) {
    setLimelightNTDouble(limelightName, LED_MODE, 2);
  }

  public static void setLEDModeForceOn(String limelightName) {
    setLimelightNTDouble(limelightName, LED_MODE, 3);
  }

  public static void setStreamModeStandard(String limelightName) {
    setLimelightNTDouble(limelightName, STREAM, 0);
  }

  public static void setStreamModePiPMain(String limelightName) {
    setLimelightNTDouble(limelightName, STREAM, 1);
  }

  public static void setStreamModePiPSecondary(String limelightName) {
    setLimelightNTDouble(limelightName, STREAM, 2);
  }

  public static void setCameraModeProcessor(String limelightName) {
    setLimelightNTDouble(limelightName, "camMode", 0);
  }

  public static void setCameraModeDriver(String limelightName) {
    setLimelightNTDouble(limelightName, "camMode", 1);
  }

  /**
   * Sets the crop window. The crop window in the UI must be completely open for dynamic cropping to
   * work.
   */
  public static void setCropWindow(
      String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
    double[] entries = new double[4];
    entries[0] = cropXMin;
    entries[1] = cropXMax;
    entries[2] = cropYMin;
    entries[3] = cropYMax;
    setLimelightNTDoubleArray(limelightName, "crop", entries);
  }

  public static void setCameraPoseRobotSpace(
      String limelightName,
      double forward,
      double side,
      double up,
      double roll,
      double pitch,
      double yaw) {
    double[] entries = new double[6];
    entries[0] = forward;
    entries[1] = side;
    entries[2] = up;
    entries[3] = roll;
    entries[4] = pitch;
    entries[5] = yaw;
    setLimelightNTDoubleArray(limelightName, "cameraposerobotspaceset", entries);
  }

  /////
  /////

  public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
    setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
  }

  public static double[] getPythonScriptData(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "llpython");
  }

  /////
  /////

  /** Asynchronously take snapshot. */
  public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
    return CompletableFuture.supplyAsync(() -> syncTakeSnapshot(tableName, snapshotName));
  }

  private static boolean syncTakeSnapshot(String tableName, String snapshotName) {
    URL url = getLimelightURLString(tableName, "capturesnapshot");
    try {
      HttpURLConnection connection = (HttpURLConnection) url.openConnection();
      connection.setRequestMethod("GET");
      if (snapshotName != null && !snapshotName.equals("")) {
        connection.setRequestProperty("snapname", snapshotName);
      }

      int responseCode = connection.getResponseCode();
      if (responseCode == 200) {
        return true;
      } else {
        DriverStation.reportError("Bad LL Request", false);
      }
    } catch (IOException e) {
      DriverStation.reportError(e.getMessage(), false);
    }
    return false;
  }

  /** Parses Limelight's JSON results dump into a LimelightResults Object */
  public static LimelightResults getLatestResults(String limelightName) {

    long start = System.nanoTime();
    LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();
    if (mapper == null) {
      mapper =
          new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    }

    try {
      results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
    } catch (JsonProcessingException e) {
      DriverStation.reportError("lljson error: " + e.getMessage(), false);
    }

    long end = System.nanoTime();
    double millis = (end - start) * .000001;
    results.targetingResults.latencyjsonParse = millis;
    if (profileJSON) {
      System.out.printf("lljson: %.2f\r%n", millis);
    }

    return results;
  }
}
