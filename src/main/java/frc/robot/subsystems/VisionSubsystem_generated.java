// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem_generated extends SubsystemBase {

    // ================================================================
    // VISION CONSTANTS (nested class)
    //
    // Think of this as the "settings panel" for all vision tuning.
    // Everything you might need to adjust after field testing is here.
    // Search for the word TUNE to jump to values you may need to tweak.
    // ================================================================
    public static class VisionConstants {

        public static final String aprilTag_cam = "AprilTag_Camera";

        // ------------------------------------------------------------
        // CAMERA POSITION ON THE ROBOT
        //
        // WPILib uses a robot-centric coordinate system:
        //   +X = toward the FRONT of the robot
        //   +Y = toward the LEFT of the robot
        //   +Z = UP
        //
        // Measured from the CENTER of the robot frame
        // (not from bumpers, not from a corner -- from the middle!).
        //
        // Your frame is 27.5" x 27.5", so center = 13.75" from each edge.
        //
        // YOUR CAMERA: // TUNE -> THESE ARE POSITION ESTIMATES
        //   Forward: 5.75" from back edge
        //            13.75 - 5.75 = 8.0" forward of center
        //            8.0 x 0.0254 = 0.2032 meters         --> kCameraForwardMeters
        //
        //   Left:    5.0" from left edge
        //            13.75 - 5.0 = 8.75" left of center
        //            8.75 x 0.0254 = 0.2223 meters        --> kCameraLeftMeters
        //
        //   Height:  19" off the ground
        //            19 x 0.0254 = 0.4826 meters           --> kCameraHeightMeters
        //
        // TUNE: If pose estimate is consistently offset re-measure and update 
        //
        // To convert any measurement: inches x 0.0254 = meters
        // ------------------------------------------------------------
        public static final double kCameraForwardMeters = 0.2032;  // 8.0 inches forward of center //TUNE!!
        public static final double kCameraLeftMeters    = 0.2223;  // 8.75 inches left of center
        public static final double kCameraHeightMeters  = 0.4826;  // 19 inches off the ground

        // ------------------------------------------------------------
        // CAMERA ROTATION
        //
        // Describes which direction the camera lens is pointing.
        // Uses Rotation3d(roll, pitch, yaw):
        //
        //   Roll  = sideways tilt. 0 unless camera is mounted rotated.
        //   Pitch = up/down tilt.
        //           0   = level (horizontal) -- your camera
        //           +   = tilted upward
        //           -   = tilted downward
        //   Yaw   = horizontal direction camera faces.
        //           0   = faces FRONT of robot
        //           90  = faces LEFT  <-- your camera (left-side mount)
        //           180 = faces BACK
        //          -90  = faces RIGHT
        //
        // TUNE: If estimated pose is rotated or mirrored, adjust yaw
        // in 90-degree steps first. If pose is offset forward/back,
        // tweak pitch in small steps (2-3 degrees at a time).
        // ------------------------------------------------------------
        public static final double kCameraRollDegrees  = 0.0;   // TUNE once mounted
        public static final double kCameraPitchDegrees = 0.0;   // TUNE: tilt up/down
        public static final double kCameraYawDegrees   = 90.0;  // Camera faces LEFT — do not change unless remounted

        // Packages position + rotation into one Transform3d object that
        // PhotonVision uses internally. Edit the values above, not this.
        // Note: WPILib pitch is inverted convention, so we negate pitch.
        public static final Transform3d kRobotToCamera = new Transform3d(
            new Translation3d(kCameraForwardMeters, kCameraLeftMeters, kCameraHeightMeters), // exact values above^^
            new Rotation3d(
                Units.degreesToRadians(kCameraRollDegrees),
                Units.degreesToRadians(-kCameraPitchDegrees), // negated: WPILib pitch is flipped
                Units.degreesToRadians(kCameraYawDegrees)
            )
        );

        // ------------------------------------------------------------
        // VISION TRUST LEVELS (Standard Deviations) <-- do we need this?? 
        //
        // The drivetrain Kalman filter blends two position sources:
        //   1. Wheel odometry (encoder-based dead reckoning)
        //   2. Vision (AprilTag-based absolute position)
        //
        // Standard deviations tell it how much to TRUST each source.
        // Format: [x error (m), y error (m), heading error (radians)]
        //
        //   SMALLER = trust MORE  (correct pose more aggressively)
        //   LARGER  = trust LESS  (only nudge the pose a little)
        //
        // kSingleTagStdDevs -- one tag visible:
        //   One tag can produce two mathematically valid poses.
        //   PhotonVision picks the better one but can be wrong.
        //   Use larger values to be cautious.
        //   TUNE: Pose jumps badly with one tag? --> increase these numbers.
        //         Single-tag seems accurate?     --> decrease these numbers.
        //
        // kMultiTagStdDevs -- two or more tags visible:
        //   Multiple tags uniquely determine position. Very reliable!
        //   Use smaller values to trust it more.
        //   TUNE: Multi-tag very accurate at your venue? --> decrease.
        //         Still seems jumpy?                     --> increase.
        // ------------------------------------------------------------

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.9, 0.9, 1.5); // potentially cut this out....
        public static final Matrix<N3, N1> kMultiTagStdDevs  = VecBuilder.fill(0.3, 0.3, 0.9);

        // ------------------------------------------------------------
        // AMBIGUITY REJECTION THRESHOLD
        // We reject single-tag readings above threshold to avoid
        // feeding garbage data into the pose estimator.
        //
        // TUNE: Pose jumping badly from single tags? --> lower (e.g. 0.15)
        //       Not getting enough vision updates?   --> raise (e.g. 0.3)
        // ------------------------------------------------------------

        public static final double kMaxAmbiguity = 0.2;

        // ------------------------------------------------------------
        //
        // GOAL tags = where you shoot fuel
        //   Red:  9, 10 (straight ahead), 8, 5 (left angle), 11, 2 (right angle) << these might be flipped
        //   Blue: 25, 26 (straight ahead), 18, 27 (left angle), 21, 24 (right angle)
        //
        // If any ID is wrong, populateTagList() will print a warning
        // to the Driver Station console at startup. Check there first
        // if targeting behaves strangely.
        // ------------------------------------------------------------
        public static final int[] kRedTowerTagIDs  = {15, 16};
        public static final int[] kBlueTowerTagIDs = {31, 32};

        public static final int[] kRedGoalTagIDs  = {9, 10, 8, 5, 11, 2};
        public static final int[] kBlueGoalTagIDs = {25, 26, 18, 27, 21, 24};

        // ------------------------------------------------------------
        // ANGLE TOLERANCE FOR TARGET SELECTION
        //
        // When choosing which tag to align to, we first check if the
        // robot is "roughly facing" any tag. This defines how wide
        // that window is. 25 degrees is a solid starting value.
        //
        // TUNE: Robot too picky and falls back to nearest too often? --> increase
        //       Robot picks wrong tags?                               --> decrease
        // ------------------------------------------------------------
        public static final double kAngTolerance = 25.0 / 180.0 * Math.PI; // 25 degrees in radians

        // ------------------------------------------------------------
        // TAG-SPECIFIC FINE-TUNE OFFSETS
        //
        // On a real competition field, individual tags are sometimes
        // mounted slightly off from where the game manual says they are.
        // This array lets you add a small per-tag correction in meters.
        //
        // HOW TO USE:
        //   Array index = Tag ID  (index 9 = Tag 9, index 15 = Tag 15, etc.)
        //   Values are in METERS, robot-centric:
        //     [0] = X offset (+  = nudge farther from the tag)
        //     [1] = Y offset (+  = nudge right relative to tag face)
        //
        // All start at 0 (no correction). Only change a specific tag's
        // values if you notice you're consistently off at that tag after
        // scoring multiple times.
        //
        // EXAMPLE: Tag 9 always leaves you 2cm too far left:
        //   Change index 9 to {0.0, 0.02}  (nudge 2cm right)
        //
        // TUNE: At competition, test each target. If you score fine,
        // leave it at 0. If consistently off in one direction, adjust
        // only that tag by small amounts (0.01-0.03m at a time).
        // ------------------------------------------------------------
        public static final double[][] tagSpecificOffset = {
            {0.0, 0.0},  // Tag  0  (unused on 2026 field)
            {0.0, 0.0},  // Tag  1
            {0.0, 0.0},  // Tag  2  (Red goal - right angle)
            {0.0, 0.0},  // Tag  3
            {0.0, 0.0},  // Tag  4
            {0.0, 0.0},  // Tag  5  (Red goal - left angle)
            {0.0, 0.0},  // Tag  6
            {0.0, 0.0},  // Tag  7
            {0.0, 0.0},  // Tag  8  (Red goal - left angle)
            {0.0, 0.0},  // Tag  9  (Red goal - straight ahead)
            {0.0, 0.0},  // Tag 10  (Red goal - straight ahead)
            {0.0, 0.0},  // Tag 11  (Red goal - right angle)
            {0.0, 0.0},  // Tag 12
            {0.0, 0.0},  // Tag 13
            {0.0, 0.0},  // Tag 14
            {0.0, 0.0},  // Tag 15  (Red tower)
            {0.0, 0.0},  // Tag 16  (Red tower)
            {0.0, 0.0},  // Tag 17
            {0.0, 0.0},  // Tag 18  (Blue goal - left angle)
            {0.0, 0.0},  // Tag 19
            {0.0, 0.0},  // Tag 20
            {0.0, 0.0},  // Tag 21  (Blue goal - right angle)
            {0.0, 0.0},  // Tag 22
            {0.0, 0.0},  // Tag 23
            {0.0, 0.0},  // Tag 24  (Blue goal - right angle)
            {0.0, 0.0},  // Tag 25  (Blue goal - straight ahead)
            {0.0, 0.0},  // Tag 26  (Blue goal - straight ahead)
            {0.0, 0.0},  // Tag 27  (Blue goal - left angle)
            {0.0, 0.0},  // Tag 28
            {0.0, 0.0},  // Tag 29
            {0.0, 0.0},  // Tag 30
            {0.0, 0.0},  // Tag 31  (Blue tower)
            {0.0, 0.0},  // Tag 32  (Blue tower)
        };
    }

    // ================================================================
    // INNER CLASS: VisionTargetTag
    //
    // A small container that holds the most recently selected target's
    // tag ID and field pose together. This way commands can ask both
    // "where do I drive?" and "which tag is it?" in one place.
    // Mirrors the same pattern used in 3940's VisionSubsystem.
    // ================================================================
    public class VisionTargetTag {
        public int tagID;    // The AprilTag ID number (e.g. 9, 15, 31...)
        public Pose2d pose;  // The 2D field position of that tag
    }


    // ================================================================
    // MEMBER VARIABLES
    // Objects used at runtime -- created once in the constructor.
    // ================================================================


    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Represents the physical OrangePi camera.
    // Must match the PhotonVision dashboard name.
    private final PhotonCamera camera;

    // Converts camera observations + field layout + camera transform
    // into a robot pose on the field.
    private final PhotonPoseEstimator photonPoseEstimator;

    // Reference to the drivetrain -- we read its current pose and
    // feed vision measurements back into it.
    private final CommandSwerveDrivetrain drivetrain;

    // Shows the robot on a field diagram in Shuffleboard.
    // Add a "Field" widget in Shuffleboard to see it!
    private final Field2d fieldDisplay = new Field2d();

    // Pre-loaded lists of tag poses for each target type + alliance.
    // Built once in the constructor from the field layout.
    private final List<Pose2d> redTowerTargets  = new ArrayList<>();
    private final List<Pose2d> blueTowerTargets = new ArrayList<>();
    private final List<Pose2d> redGoalTargets   = new ArrayList<>();
    private final List<Pose2d> blueGoalTargets  = new ArrayList<>();

    // The most recently selected target (updated by getClosest* methods).
    // Commands can read tag ID + pose from this via public getters.
    private VisionTargetTag currentTarget = new VisionTargetTag();

    /**
     * The most recently processed PhotonPipelineResult, cached by periodic().
     *
     * WHY THIS EXISTS: PhotonCamera.getAllUnreadResults() consumes results from
     * an internal buffer — once read, they're gone. If both the vision subsystem
     * (periodic) AND a command (execute) both called getAllUnreadResults(), one
     * of them would always get an empty list. By caching here and exposing via
     * getLatestResult(), both can safely read the same frame.
     *
     * AlignToTagCommand reads this via getLatestResult() each execute() loop.
     */
    private PhotonPipelineResult latestCachedResult = null;


    // ================================================================
    // CONSTRUCTOR
    //
    // Called once at robot startup (from RobotContainer).
    // Sets everything up: camera, pose estimator, field display, tags.
    // ================================================================
    public VisionSubsystem_generated(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Load the 2026 FRC field's AprilTag layout from WPILib's
        // built-in resources. This gives us the 3D position of every
        // tag on the official field.
        //
        // NOTE: kDefaultField always refers to the CURRENT year's field.
        // If WPILib hasn't released the 2026 layout yet at kickoff,
        // you may need to load a custom JSON file instead. Ask your
        // programming mentor or check WPILib release notes for the year.
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Create the camera object using our camera name constant.
        // REMINDER: name must match PhotonVision dashboard exactly.
        // If it doesn't match, you'll get no results and no error message!
        camera = new PhotonCamera(VisionConstants.aprilTag_cam);

        // Create the pose estimator.
        //
        // MULTI_TAG_PNP_ON_COPROCESSOR:
        //   The best strategy available. Uses ALL visible tags together
        //   to calculate one accurate robot pose. The heavy math runs
        //   on the OrangePi (not the RoboRIO), so your robot loop stays fast.
        //
        // Fallback = CLOSEST_TO_REFERENCE_POSE:
        //   If multi-tag fails (e.g. weird tag geometry), fall back to
        //   the single-tag result closest to our last known pose rather
        //   than throwing the measurement out entirely.
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kRobotToCamera
        );
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        // Add the field display widget to SmartDashboard.
        // In Shuffleboard: click "+", search "Field", add widget.
        // You'll see the robot moving on the field in real time!
        SmartDashboard.putData("Field", fieldDisplay);

        // Hook into PathPlanner logging so the active auto path and
        // current target pose appear on the field display.
        // Great for debugging why your auto went to the wrong place!
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            fieldDisplay.getObject("target pose").setPose(targetPose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            fieldDisplay.getObject("path").setPoses(poses);
        });

        // Pre-load all the target tag poses from the field layout.
        // We do this ONCE here instead of every 20ms loop for speed.
        // If any tag ID doesn't exist, a warning prints to DS console.
        populateTagList(redTowerTargets,  VisionConstants.kRedTowerTagIDs);
        populateTagList(blueTowerTargets, VisionConstants.kBlueTowerTagIDs);
        populateTagList(redGoalTargets,   VisionConstants.kRedGoalTagIDs);
        populateTagList(blueGoalTargets,  VisionConstants.kBlueGoalTagIDs);
    }


    // ================================================================
    // PERIODIC
    //
    // Called automatically every 20ms by WPILib's command scheduler.
    // This is the heartbeat of the subsystem.
    // ================================================================
    @Override
    public void periodic() {
        // Before processing new camera data, tell the pose estimator
        // our current known robot pose as a reference point.
        // This helps it resolve ambiguity when only one tag is visible --
        // it picks whichever result is closer to where we already are.
        photonPoseEstimator.setReferencePose(drivetrain.getState().Pose);

        // Process latest camera frames and update the drivetrain pose
        updatePoseEstimate();

        // Keep the field display showing the current robot position
        fieldDisplay.setRobotPose(drivetrain.getState().Pose);
    }


    // ================================================================
    // PRIVATE: updatePoseEstimate()
    //
    // The core of pose estimation. Every 20ms this method:
    //   1. Grabs the latest camera frame(s)
    //   2. Skips if nothing is visible
    //   3. Rejects high-ambiguity single-tag readings
    //   4. Asks the estimator to compute a robot pose
    //   5. Decides how much to trust it (based on tag count)
    //   6. Feeds it into the drivetrain's Kalman filter
    //   7. Updates the Shuffleboard field display
    // ================================================================
    private void updatePoseEstimate() {
        // Get all frames captured since last time we called this.
        // Camera may run faster than 50Hz so there could be multiple.
        // We only want the freshest one -- stale frames aren't useful.
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return; // No new frames since last loop, nothing to do
        }

        // Only process the most recent frame
        PhotonPipelineResult latestResult = results.get(results.size() - 1);

        // Cache this result so AlignToTagCommand can read it via getLatestResult()
        // without calling getAllUnreadResults() again (which would get an empty list).
        latestCachedResult = latestResult;

        // No AprilTags in view -- push estimate marker off-screen so
        // it's obvious in Shuffleboard that we have no vision data.
        if (!latestResult.hasTargets()) {
            fieldDisplay.getObject("Vision Estimate").setPose(
                new Pose2d(-10, -10, new Rotation2d())
            );
            SmartDashboard.putString("Vision/Status", "No targets visible");
            return;
        }

        // For single-tag results: check ambiguity score.
        // High ambiguity = camera isn't sure which of two possible
        // poses is the right one. Better to skip than guess wrong.
        if (latestResult.getTargets().size() == 1) {
            double ambiguity = latestResult.getBestTarget().getPoseAmbiguity();
            if (ambiguity > VisionConstants.kMaxAmbiguity) {
                SmartDashboard.putString("Vision/Status",
                    "Rejected: ambiguity=" + String.format("%.2f", ambiguity));
                return;
            }
        }

        // Ask the estimator to compute a robot pose from this frame.
        // It uses: known tag positions + camera transform + what it sees.
        // Returns Optional.empty() if it can't produce a valid result
        // (e.g. tag not in layout, weird geometry, etc.)
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(latestResult);

        if (estimatedPose.isEmpty()) {
            SmartDashboard.putString("Vision/Status", "Estimator gave no result");
            return;
        }

        // Pick trust level based on how many tags were visible.
        // More tags = more accurate = trust the measurement more.
        int tagCount = latestResult.getTargets().size();
        Matrix<N3, N1> stdDevs;
        if (tagCount >= 2) {
            stdDevs = VisionConstants.kMultiTagStdDevs;
            SmartDashboard.putString("Vision/Status", "Good! " + tagCount + " tags visible");
        } else {
            stdDevs = VisionConstants.kSingleTagStdDevs;
            SmartDashboard.putString("Vision/Status", "OK - 1 tag visible");
        }

        // Feed the vision pose into the drivetrain's Kalman filter.
        //
        // drivetrain.addVisionMeasurement() is defined in your
        // CommandSwerveDrivetrain.java -- it automatically converts
        // the PhotonVision FPGA timestamp using Utils.fpgaToCurrentTime().
        //
        // The Kalman filter blends this with wheel odometry using the
        // standard deviations we pass in. Result: better position than
        // either source alone.
        drivetrain.addVisionMeasurement(
            estimatedPose.get().estimatedPose.toPose2d(), // convert 3D pose to 2D field pose
            estimatedPose.get().timestampSeconds,          // when this photo was taken
            stdDevs                                        // how much to trust it
        );

        // Show the camera's pose estimate on the field display.
        // DEBUGGING TIP: If this dot is far from the robot outline
        // in Shuffleboard, your camera transform is wrong. Re-check:
        //   kCameraForwardMeters, kCameraLeftMeters, kCameraYawDegrees
        fieldDisplay.getObject("Vision Estimate").setPose(
            estimatedPose.get().estimatedPose.toPose2d()
        );

        // Publish debug values to SmartDashboard for troubleshooting
        SmartDashboard.putNumber("Vision/EstimatedX", estimatedPose.get().estimatedPose.getX());
        SmartDashboard.putNumber("Vision/EstimatedY", estimatedPose.get().estimatedPose.getY());
        SmartDashboard.putNumber("Vision/TagCount",   tagCount);
    }


    // ================================================================
    // PUBLIC: getLatestResult()
    //
    // Returns the most recently cached PhotonPipelineResult from
    // the last periodic() call.
    //
    // Used by AlignToTagCommand to read camera data without creating
    // a second PhotonCamera object (which would cause dropped results).
    //
    // Returns null if no frame has been received yet since robot startup.
    // AlignToTagCommand always null-checks before using this value.
    // ================================================================
    public PhotonPipelineResult getLatestResult() {
        return latestCachedResult;
    }


    // ================================================================
    // PUBLIC: getClosestGoalTarget(boolean isBlueAlliance)
    //
    // Returns the Pose2d of the best Goal tag for the robot to aim at
    // when shooting fuel. Includes all goal angles so the robot can
    // score from wherever it currently is on the field.
    //
    // HOW TO CALL FROM A COMMAND:
    //   boolean isBlue = DriverStation.getAlliance()
    //       .orElse(Alliance.Blue) == Alliance.Blue;
    //   Pose2d goalPose = visionSubsystem.getClosestGoalTarget(isBlue);
    //   // Then use PathPlanner on-the-fly path or PID to drive there
    // ================================================================
    public Pose2d getClosestGoalTarget(boolean isBlueAlliance) {
        List<Pose2d> targets = isBlueAlliance ? blueGoalTargets : redGoalTargets;
        int[] tagIDs         = isBlueAlliance
            ? VisionConstants.kBlueGoalTagIDs
            : VisionConstants.kRedGoalTagIDs;
        return findBestTarget(targets, tagIDs);
    }


    // ================================================================
    // PUBLIC: getClosestTowerTarget(boolean isBlueAlliance)
    //
    // Returns the Pose2d of the best Tower tag to align to for climbing.
    //
    // HOW TO CALL FROM A COMMAND:
    //   boolean isBlue = DriverStation.getAlliance()
    //       .orElse(Alliance.Blue) == Alliance.Blue;
    //   Pose2d towerPose = visionSubsystem.getClosestTowerTarget(isBlue);
    // ================================================================
    public Pose2d getClosestTowerTarget(boolean isBlueAlliance) {
        List<Pose2d> targets = isBlueAlliance ? blueTowerTargets : redTowerTargets;
        int[] tagIDs         = isBlueAlliance
            ? VisionConstants.kBlueTowerTagIDs
            : VisionConstants.kRedTowerTagIDs;
        return findBestTarget(targets, tagIDs);
    }


    // ================================================================
    // PUBLIC: getTargetTagID()
    //
    // Returns the tag ID that was selected by the most recent call to
    // getClosestGoalTarget() or getClosestTowerTarget().
    //
    // Use this if a command needs to look up a tag-specific offset:
    //   int id = visionSubsystem.getTargetTagID();
    //   double xOffset = VisionConstants.tagSpecificOffset[id][0];
    //   double yOffset = VisionConstants.tagSpecificOffset[id][1];
    //
    // Returns 0 if no target has been selected yet this match.
    // ================================================================
    public int getTargetTagID() {
        return currentTarget.tagID;
    }


    // ================================================================
    // PUBLIC: getTagPose(int tagID)
    //
    // Returns the known 3D field position of any AprilTag by ID.
    // Useful for advanced math or debugging.
    // Throws RuntimeException if the ID doesn't exist in the layout.
    // ================================================================
    public Pose3d getTagPose(int tagID) {
        return aprilTagFieldLayout.getTagPose(tagID)
            .orElseThrow(() -> new RuntimeException(
                "[VisionSubsystem] Tag ID " + tagID
                + " not found in field layout! Check tag IDs in VisionConstants."));
    }


    // ================================================================
    // PRIVATE: findBestTarget(targets, tagIDs)
    //
    // Core target selection logic -- directly based on 3940's
    // getClosestReefTargetRed/Blue() methods.
    //
    // STEP 1: Is the robot roughly facing any target?
    //   Loop through all targets. If the robot is pointing at one
    //   within kAngTolerance, immediately return that one.
    //   (Front-facing camera: we check the FRONT direction.
    //    3940 used rear direction because cameras faced backward.)
    //
    // STEP 2: Not facing any target?
    //   Fall back to the nearest one by distance.
    //
    // The "facing" check uses a cosine trick to avoid messy angle
    // wraparound math at ±180 degrees -- same as 3940's code.
    // ================================================================
    private Pose2d findBestTarget(List<Pose2d> targets, int[] tagIDs) {
        // Safety: shouldn't happen if tag IDs are right, but beats
        // a NullPointerException crashing the robot mid-match.
        if (targets.isEmpty()) {
            System.out.println("[VisionSubsystem] WARNING: target list is empty!"
                + " Check tag IDs in VisionConstants.");
            currentTarget.tagID = 0;
            currentTarget.pose  = new Pose2d();
            return currentTarget.pose;
        }

        // Get where the robot currently is on the field
        Pose2d robotPose = drivetrain.getState().Pose;

        // The direction the FRONT of the robot is pointing (radians).
        // Camera faces forward so this is what we compare tag angles to.
        double robotFrontDirection = robotPose.getRotation().getRadians();

        // STEP 1: Find a target the robot is roughly facing
        for (int i = 0; i < targets.size(); i++) {
            // Each tag pose's rotation describes which direction that tag
            // is facing (i.e., the direction a robot would approach from).
            double tagFacingAngle = targets.get(i).getRotation().getRadians();

            // Cosine trick (same as 3940):
            // Instead of computing (robotAngle - tagAngle) and checking if
            // it's within tolerance (which breaks at the ±180 boundary),
            // we use: cos(diff) > cos(tolerance)
            // This is mathematically equivalent but handles wraparound cleanly.
            boolean facingTarget =
                Math.cos(tagFacingAngle - robotFrontDirection)
                > Math.cos(VisionConstants.kAngTolerance);

            if (facingTarget) {
                // Robot is roughly facing this tag -- use it!
                currentTarget.tagID = tagIDs[i];
                currentTarget.pose  = targets.get(i);
                SmartDashboard.putNumber("Vision/TargetTagID", currentTarget.tagID);
                return currentTarget.pose;
            }
        }

        // STEP 2: Not facing any target -- use the nearest one.
        // Pose2d.nearest() returns the closest pose in the list.
        currentTarget.pose = robotPose.nearest(targets);

        // Find which index in the list is the nearest pose,
        // so we can record the corresponding tag ID.
        for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getX() == currentTarget.pose.getX()
             && targets.get(i).getY() == currentTarget.pose.getY()) {
                currentTarget.tagID = tagIDs[i];
                break;
            }
        }

        SmartDashboard.putNumber("Vision/TargetTagID", currentTarget.tagID);
        return currentTarget.pose;
    }


    // ================================================================
    // PRIVATE: populateTagList(list, tagIDs)
    //
    // Looks up each tag ID in the field layout and adds its 2D pose
    // to the given list. Called once in the constructor.
    //
    // If a tag ID doesn't exist in the layout, prints a warning to
    // the Driver Station console and skips it. You'll see this at
    // startup if any tag IDs in VisionConstants are wrong.
    // ================================================================
    private void populateTagList(List<Pose2d> list, int[] tagIDs) {
        for (int id : tagIDs) {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(id);
            if (tagPose.isPresent()) {
                list.add(tagPose.get().toPose2d());
            } else {
                // Printed to Driver Station console at startup.
                // Fix the tag ID in VisionConstants if you see this!
                System.out.println("[VisionSubsystem] WARNING: Tag ID " + id
                    + " not found in the 2026 field layout!"
                    + " Update the tag ID arrays in VisionConstants.");
            }
        }
    }
}



