package frc.robot.vision;

//LimelightHelpers v1.2.1 (March 1, 2023)

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

public class LimelightHelpers {

	public static class LimelightTarget_Retro {

		@JsonProperty("t6c_ts")
		private final double[] cameraPose_TargetSpace;

		@JsonProperty("t6r_fs")
		private final double[] robotPose_FieldSpace;

		@JsonProperty("t6r_ts")
		private final double[] robotPose_TargetSpace;

		@JsonProperty("t6t_cs")
		private final double[] targetPose_CameraSpace;

		@JsonProperty("t6t_rs")
		private final double[] targetPose_RobotSpace;

		public Pose3d getCameraPose_TargetSpace() { return LimelightHelpers.toPose3D(this.cameraPose_TargetSpace); }

		public Pose3d getRobotPose_FieldSpace() { return LimelightHelpers.toPose3D(this.robotPose_FieldSpace); }

		public Pose3d getRobotPose_TargetSpace() { return LimelightHelpers.toPose3D(this.robotPose_TargetSpace); }

		public Pose3d getTargetPose_CameraSpace() { return LimelightHelpers.toPose3D(this.targetPose_CameraSpace); }

		public Pose3d getTargetPose_RobotSpace() { return LimelightHelpers.toPose3D(this.targetPose_RobotSpace); }

		public Pose2d getCameraPose_TargetSpace2D() { return LimelightHelpers.toPose2D(this.cameraPose_TargetSpace); }

		public Pose2d getRobotPose_FieldSpace2D() { return LimelightHelpers.toPose2D(this.robotPose_FieldSpace); }

		public Pose2d getRobotPose_TargetSpace2D() { return LimelightHelpers.toPose2D(this.robotPose_TargetSpace); }

		public Pose2d getTargetPose_CameraSpace2D() { return LimelightHelpers.toPose2D(this.targetPose_CameraSpace); }

		public Pose2d getTargetPose_RobotSpace2D() { return LimelightHelpers.toPose2D(this.targetPose_RobotSpace); }

		@JsonProperty("ta")
		public double ta;

		@JsonProperty("tx")
		public double tx;

		@JsonProperty("txp")
		public double tx_pixels;

		@JsonProperty("ty")
		public double ty;

		@JsonProperty("typ")
		public double ty_pixels;

		@JsonProperty("ts")
		public double ts;

		public LimelightTarget_Retro() {
			this.cameraPose_TargetSpace = new double[6];
			this.robotPose_FieldSpace = new double[6];
			this.robotPose_TargetSpace = new double[6];
			this.targetPose_CameraSpace = new double[6];
			this.targetPose_RobotSpace = new double[6];
		}

	}

	public static class LimelightTarget_Fiducial {

		@JsonProperty("fID")
		public double fiducialID;

		@JsonProperty("fam")
		public String fiducialFamily;

		@JsonProperty("t6c_ts")
		private final double[] cameraPose_TargetSpace;

		@JsonProperty("t6r_fs")
		private final double[] robotPose_FieldSpace;

		@JsonProperty("t6r_ts")
		private final double[] robotPose_TargetSpace;

		@JsonProperty("t6t_cs")
		private final double[] targetPose_CameraSpace;

		@JsonProperty("t6t_rs")
		private final double[] targetPose_RobotSpace;

		public Pose3d getCameraPose_TargetSpace() { return LimelightHelpers.toPose3D(this.cameraPose_TargetSpace); }

		public Pose3d getRobotPose_FieldSpace() { return LimelightHelpers.toPose3D(this.robotPose_FieldSpace); }

		public Pose3d getRobotPose_TargetSpace() { return LimelightHelpers.toPose3D(this.robotPose_TargetSpace); }

		public Pose3d getTargetPose_CameraSpace() { return LimelightHelpers.toPose3D(this.targetPose_CameraSpace); }

		public Pose3d getTargetPose_RobotSpace() { return LimelightHelpers.toPose3D(this.targetPose_RobotSpace); }

		public Pose2d getCameraPose_TargetSpace2D() { return LimelightHelpers.toPose2D(this.cameraPose_TargetSpace); }

		public Pose2d getRobotPose_FieldSpace2D() { return LimelightHelpers.toPose2D(this.robotPose_FieldSpace); }

		public Pose2d getRobotPose_TargetSpace2D() { return LimelightHelpers.toPose2D(this.robotPose_TargetSpace); }

		public Pose2d getTargetPose_CameraSpace2D() { return LimelightHelpers.toPose2D(this.targetPose_CameraSpace); }

		public Pose2d getTargetPose_RobotSpace2D() { return LimelightHelpers.toPose2D(this.targetPose_RobotSpace); }

		@JsonProperty("ta")
		public double ta;

		@JsonProperty("tx")
		public double tx;

		@JsonProperty("txp")
		public double tx_pixels;

		@JsonProperty("ty")
		public double ty;

		@JsonProperty("typ")
		public double ty_pixels;

		@JsonProperty("ts")
		public double ts;

		public LimelightTarget_Fiducial() {
			this.cameraPose_TargetSpace = new double[6];
			this.robotPose_FieldSpace = new double[6];
			this.robotPose_TargetSpace = new double[6];
			this.targetPose_CameraSpace = new double[6];
			this.targetPose_RobotSpace = new double[6];
		}
	}

	public static class LimelightTarget_Barcode {

	}

	public static class LimelightTarget_Classifier {

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
		public double tx_pixels;

		@JsonProperty("ty")
		public double ty;

		@JsonProperty("typ")
		public double ty_pixels;

		public LimelightTarget_Classifier() {}
	}

	public static class LimelightTarget_Detector {

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
		public double tx_pixels;

		@JsonProperty("ty")
		public double ty;

		@JsonProperty("typ")
		public double ty_pixels;

		public LimelightTarget_Detector() {}
	}

	public static class Results {

		@JsonProperty("pID")
		public double pipelineID;

		@JsonProperty("tl")
		public double latency_pipeline;

		@JsonProperty("cl")
		public double latency_capture;

		public double latency_jsonParse;

		@JsonProperty("ts")
		public double timestamp_LIMELIGHT_publish;

		@JsonProperty("ts_rio")
		public double timestamp_RIOFPGA_capture;

		@JsonProperty("v")
		@JsonFormat(shape = Shape.NUMBER)
		public boolean valid;

		@JsonProperty("botpose")
		public double[] botpose;

		@JsonProperty("botpose_wpired")
		public double[] botpose_wpired;

		@JsonProperty("botpose_wpiblue")
		public double[] botpose_wpiblue;

		@JsonProperty("t6c_rs")
		public double[] camerapose_robotspace;

		public Pose3d getBotPose3d() { return LimelightHelpers.toPose3D(this.botpose); }

		public Pose3d getBotPose3d_wpiRed() { return LimelightHelpers.toPose3D(this.botpose_wpired); }

		public Pose3d getBotPose3d_wpiBlue() { return LimelightHelpers.toPose3D(this.botpose_wpiblue); }

		public Pose2d getBotPose2d() { return LimelightHelpers.toPose2D(this.botpose); }

		public Pose2d getBotPose2d_wpiRed() { return LimelightHelpers.toPose2D(this.botpose_wpired); }

		public Pose2d getBotPose2d_wpiBlue() { return LimelightHelpers.toPose2D(this.botpose_wpiblue); }

		@JsonProperty("Retro")
		public LimelightTarget_Retro[] targets_Retro;

		@JsonProperty("Fiducial")
		public LimelightTarget_Fiducial[] targets_Fiducials;

		@JsonProperty("Classifier")
		public LimelightTarget_Classifier[] targets_Classifier;

		@JsonProperty("Detector")
		public LimelightTarget_Detector[] targets_Detector;

		@JsonProperty("Barcode")
		public LimelightTarget_Barcode[] targets_Barcode;

		public Results() {
			this.botpose = new double[6];
			this.botpose_wpired = new double[6];
			this.botpose_wpiblue = new double[6];
			this.camerapose_robotspace = new double[6];
			this.targets_Retro = new LimelightTarget_Retro[0];
			this.targets_Fiducials = new LimelightTarget_Fiducial[0];
			this.targets_Classifier = new LimelightTarget_Classifier[0];
			this.targets_Detector = new LimelightTarget_Detector[0];
			this.targets_Barcode = new LimelightTarget_Barcode[0];

		}
	}

	public static class LimelightResults {
		@JsonProperty("Results")
		public Results targetingResults;

		public LimelightResults() { this.targetingResults = new Results(); }
	}

	private static ObjectMapper mapper;

	/**
	 * Print JSON Parse time to the console in milliseconds
	 */
	static boolean profileJSON = false;

	static final String sanitizeName(final String name) {
		if(name == "" || name == null) {
			return "limelight";
		}
		return name;
	}

	private static Pose3d toPose3D(final double[] inData) {
		if(inData.length < 6) {
			// System.err.println("Bad LL 3D Pose Data!");
			return new Pose3d();
		}
		return new Pose3d(
			new Translation3d(inData[0], inData[1], inData[2]),
			new Rotation3d(
				Units.degreesToRadians(inData[3]),
				Units.degreesToRadians(inData[4]),
				Units.degreesToRadians(inData[5])
			)
		);
	}

	private static Pose2d toPose2D(final double[] inData) {
		if(inData.length < 6) {
			// System.err.println("Bad LL 2D Pose Data!");
			return new Pose2d();
		}
		final Translation2d tran2d = new Translation2d(inData[0], inData[1]);
		final Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
		return new Pose2d(tran2d, r2d);
	}

	public static NetworkTable getLimelightNTTable(final String tableName) {
		return NetworkTableInstance.getDefault().getTable(LimelightHelpers.sanitizeName(tableName));
	}

	public static NetworkTableEntry getLimelightNTTableEntry(final String tableName, final String entryName) {
		return LimelightHelpers.getLimelightNTTable(tableName).getEntry(entryName);
	}

	public static double getLimelightNTDouble(final String tableName, final String entryName) {
		return LimelightHelpers.getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
	}

	public static void setLimelightNTDouble(final String tableName, final String entryName, final double val) {
		LimelightHelpers.getLimelightNTTableEntry(tableName, entryName).setDouble(val);
	}

	public static void setLimelightNTDoubleArray(final String tableName, final String entryName, final double[] val) {
		LimelightHelpers.getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
	}

	public static double[] getLimelightNTDoubleArray(final String tableName, final String entryName) {
		return LimelightHelpers.getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
	}

	public static String getLimelightNTString(final String tableName, final String entryName) {
		return LimelightHelpers.getLimelightNTTableEntry(tableName, entryName).getString("");
	}

	public static URL getLimelightURLString(final String tableName, final String request) {
		final String urlString = "http://" + LimelightHelpers.sanitizeName(tableName) + ".local:5807/" + request;
		URL url;
		try {
			url = new URL(urlString);
			return url;
		} catch(final MalformedURLException e) {
			System.err.println("bad LL URL");
		}
		return null;
	}
	/////
	/////

	public static double getTX(final String limelightName) {
		return LimelightHelpers.getLimelightNTDouble(limelightName, "tx");
	}

	public static double getTY(final String limelightName) {
		return LimelightHelpers.getLimelightNTDouble(limelightName, "ty");
	}

	public static double getTA(final String limelightName) {
		return LimelightHelpers.getLimelightNTDouble(limelightName, "ta");
	}

	public static double getLatency_Pipeline(final String limelightName) {
		return LimelightHelpers.getLimelightNTDouble(limelightName, "tl");
	}

	public static double getLatency_Capture(final String limelightName) {
		return LimelightHelpers.getLimelightNTDouble(limelightName, "cl");
	}

	public static double getCurrentPipelineIndex(final String limelightName) {
		return LimelightHelpers.getLimelightNTDouble(limelightName, "getpipe");
	}

	public static String getJSONDump(final String limelightName) {
		return LimelightHelpers.getLimelightNTString(limelightName, "json");
	}

	/**
	 * Switch to getBotPose
	 *
	 * @param limelightName
	 * @return
	 */
	@Deprecated
	public static double[] getBotpose(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose");
	}

	/**
	 * Switch to getBotPose_wpiRed
	 *
	 * @param limelightName
	 * @return
	 */
	@Deprecated
	public static double[] getBotpose_wpiRed(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_wpired");
	}

	/**
	 * Switch to getBotPose_wpiBlue
	 *
	 * @param limelightName
	 * @return
	 */
	@Deprecated
	public static double[] getBotpose_wpiBlue(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
	}

	public static double[] getBotPose(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose");
	}

	public static double[] getBotPose_wpiRed(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_wpired");
	}

	public static double[] getBotPose_wpiBlue(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
	}

	public static double[] getBotPose_TargetSpace(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
	}

	public static double[] getCameraPose_TargetSpace(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
	}

	public static double[] getTargetPose_CameraSpace(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
	}

	public static double[] getTargetPose_RobotSpace(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
	}

	public static double[] getTargetColor(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "tc");
	}

	public static double getFiducialID(final String limelightName) {
		return LimelightHelpers.getLimelightNTDouble(limelightName, "tid");
	}

	public static double getNeuralClassID(final String limelightName) {
		return LimelightHelpers.getLimelightNTDouble(limelightName, "tclass");
	}

	/////
	/////

	public static Pose3d getBotPose3d(final String limelightName) {
		final double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose");
		return LimelightHelpers.toPose3D(poseArray);
	}

	public static Pose3d getBotPose3d_wpiRed(final String limelightName) {
		final double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_wpired");
		return LimelightHelpers.toPose3D(poseArray);
	}

	public static Pose3d getBotPose3d_wpiBlue(final String limelightName) {
		final double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
		return LimelightHelpers.toPose3D(poseArray);
	}

	public static Pose3d getBotPose3d_TargetSpace(final String limelightName) {
		final double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
		return LimelightHelpers.toPose3D(poseArray);
	}

	public static Pose3d getCameraPose3d_TargetSpace(final String limelightName) {
		final double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
		return LimelightHelpers.toPose3D(poseArray);
	}

	public static Pose3d getTargetPose3d_CameraSpace(final String limelightName) {
		final double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
		return LimelightHelpers.toPose3D(poseArray);
	}

	public static Pose3d getTargetPose3d_RobotSpace(final String limelightName) {
		final double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
		return LimelightHelpers.toPose3D(poseArray);
	}

	public static Pose3d getCameraPose3d_RobotSpace(final String limelightName) {
		final double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
		return LimelightHelpers.toPose3D(poseArray);
	}

	/**
	 * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
	 *
	 * @param limelightName
	 * @return
	 */
	public static Pose2d getBotPose2d_wpiBlue(final String limelightName) {

		final double[] result = LimelightHelpers.getBotPose_wpiBlue(limelightName);
		return LimelightHelpers.toPose2D(result);
	}

	/**
	 * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
	 *
	 * @param limelightName
	 * @return
	 */
	public static Pose2d getBotPose2d_wpiRed(final String limelightName) {

		final double[] result = LimelightHelpers.getBotPose_wpiRed(limelightName);
		return LimelightHelpers.toPose2D(result);

	}

	/**
	 * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
	 *
	 * @param limelightName
	 * @return
	 */
	public static Pose2d getBotPose2d(final String limelightName) {

		final double[] result = LimelightHelpers.getBotPose(limelightName);
		return LimelightHelpers.toPose2D(result);

	}

	public static boolean getTV(final String limelightName) {
		return 1.0 == LimelightHelpers.getLimelightNTDouble(limelightName, "tv");
	}

	/////
	/////

	public static void setPipelineIndex(final String limelightName, final int pipelineIndex) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
	}

	/**
	 * The LEDs will be controlled by Limelight pipeline settings, and not by robot code.
	 */
	public static void setLEDMode_PipelineControl(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "ledMode", 0);
	}

	public static void setLEDMode_ForceOff(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "ledMode", 1);
	}

	public static void setLEDMode_ForceBlink(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "ledMode", 2);
	}

	public static void setLEDMode_ForceOn(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "ledMode", 3);
	}

	public static void setStreamMode_Standard(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "stream", 0);
	}

	public static void setStreamMode_PiPMain(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "stream", 1);
	}

	public static void setStreamMode_PiPSecondary(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "stream", 2);
	}

	public static void setCameraMode_Processor(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "camMode", 0);
	}

	public static void setCameraMode_Driver(final String limelightName) {
		LimelightHelpers.setLimelightNTDouble(limelightName, "camMode", 1);
	}

	/**
	 * Sets the crop window. The crop window in the UI must be completely open for dynamic cropping to work.
	 */
	public static void setCropWindow(
		final String limelightName,
		final double cropXMin,
		final double cropXMax,
		final double cropYMin,
		final double cropYMax
	) {
		final double[] entries = new double[4];
		entries[0] = cropXMin;
		entries[1] = cropXMax;
		entries[2] = cropYMin;
		entries[3] = cropYMax;
		LimelightHelpers.setLimelightNTDoubleArray(limelightName, "crop", entries);
	}

	public static void setCameraPose_RobotSpace(
		final String limelightName,
		final double forward,
		final double side,
		final double up,
		final double roll,
		final double pitch,
		final double yaw
	) {
		final double[] entries = new double[6];
		entries[0] = forward;
		entries[1] = side;
		entries[2] = up;
		entries[3] = roll;
		entries[4] = pitch;
		entries[5] = yaw;
		LimelightHelpers.setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
	}

	/////
	/////

	public static void setPythonScriptData(final String limelightName, final double[] outgoingPythonData) {
		LimelightHelpers.setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
	}

	public static double[] getPythonScriptData(final String limelightName) {
		return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "llpython");
	}

	/////
	/////

	/**
	 * Asynchronously take snapshot.
	 */
	public static CompletableFuture<Boolean> takeSnapshot(final String tableName, final String snapshotName) {
		return CompletableFuture.supplyAsync(() -> LimelightHelpers.SYNCH_TAKESNAPSHOT(tableName, snapshotName));
	}

	private static boolean SYNCH_TAKESNAPSHOT(final String tableName, final String snapshotName) {
		final URL url = LimelightHelpers.getLimelightURLString(tableName, "capturesnapshot");
		try {
			final HttpURLConnection connection = (HttpURLConnection) url.openConnection();
			connection.setRequestMethod("GET");
			if(snapshotName != null && snapshotName != "") {
				connection.setRequestProperty("snapname", snapshotName);
			}

			final int responseCode = connection.getResponseCode();
			if(responseCode == 200) {
				return true;
			} else {
				System.err.println("Bad LL Request");
			}
		} catch(final IOException e) {
			System.err.println(e.getMessage());
		}
		return false;
	}

	/**
	 * Parses Limelight's JSON results dump into a LimelightResults Object
	 */
	public static LimelightResults getLatestResults(final String limelightName) {

		final long start = System.nanoTime();
		LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();
		if(LimelightHelpers.mapper == null) {
			LimelightHelpers.mapper = new ObjectMapper()
				.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
		}

		try {
			results = LimelightHelpers.mapper
				.readValue(LimelightHelpers.getJSONDump(limelightName), LimelightResults.class);
		} catch(final JsonProcessingException e) {
			System.err.println("lljson error: " + e.getMessage());
		}

		final long end = System.nanoTime();
		final double millis = (end - start) * .000001;
		results.targetingResults.latency_jsonParse = millis;
		if(LimelightHelpers.profileJSON) {
			System.out.printf("lljson: %.2f\r\n", millis);
		}

		return results;
	}
}
