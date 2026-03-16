// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem_generated;
import frc.robot.subsystems.VisionSubsystem_generated.VisionConstants;

/**
 * AlignToTagCommand
 *
 * Fully aligns the robot with the nearest valid alliance AprilTag using
 * three independent PID controllers running simultaneously:
 *
 *   1. ROTATION  — Rotates the robot until the tag is centered in the
 *                  left-side camera's FOV (tag yaw = 0), meaning the
 *                  robot's left bumper is perpendicular to the tag face.
 *
 *   2. FORWARD   — Drives the robot forward/backward until the tag has
 *                  no lateral offset along the tag face (robot X error = 0),
 *                  centering the robot directly across from the tag.
 *
 *   3. STRAFE    — Drives the robot left/right until the perpendicular
 *                  distance to the tag equals TARGET_DISTANCE_METERS.
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  CAMERA PLACEMENT NOTE                                          │
 * │  Camera is on the LEFT side of the robot, facing LEFT.         │
 * │  "Aligned" = robot's left side is TARGET_DISTANCE from tag,    │
 * │              robot center is laterally centered on the tag.    │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * Red alliance valid tag IDs  : 8, 5, 9, 10, 11, 2
 * Blue alliance valid tag IDs : 18, 27, 26, 25, 24, 21
 *
 * TRIGGER: Hold the assigned button (whileTrue in RobotContainer).
 *          All motion stops immediately when the button is released.
 *
 * REQUIREMENTS: CommandSwerveDrivetrain (owns swerve),
 *               VisionSubsystem_generated (owns the PhotonCamera).
 *
 * HOW TO BIND (in RobotContainer.configureBindings):
 *   driverController.leftBumper().whileTrue(
 *       new AlignToTagCommand(drivetrain, visionSubsystem_generated));
 */
public class AlignToTagCommand extends Command {

    // =================================================================
    // TUNABLE CONSTANTS
    // Search for "TUNE" to find every value you may want to adjust.
    // =================================================================

    /**
     * Distance (meters) the robot's left side should be from the tag face
     * when fully aligned. Measure from your bumper to the tag surface.
     *
     * TUNE: Start at 0.5 m, adjust in 0.05 m steps during practice.
     */
    private static final double TARGET_DISTANCE_METERS = 0.5; // TUNE

    // -----------------------------------------------------------------
    // Rotation PID  (drives tag yaw error → 0 degrees)
    // Input : target.getYaw()  [degrees]
    // Output: rotational rate  [radians/sec, applied to swerve]
    //
    // TUNE kP first. Increase until the robot turns briskly without
    //      oscillating. Then add a small kD to damp overshoots.
    // -----------------------------------------------------------------
    private static final double ROTATION_kP        = 0.035; // TUNE  (deg → rad/s)
    private static final double ROTATION_kI        = 0.0;   // TUNE  (usually leave 0)
    private static final double ROTATION_kD        = 0.002; // TUNE  (damp oscillation)
    private static final double ROTATION_TOLERANCE = 1.5;   // TUNE  degrees

    // -----------------------------------------------------------------
    // Forward PID  (drives robot-frame X error → 0 meters)
    // Input : robot-to-target X component  [meters]
    // Output: forward velocity             [m/s]
    //
    // TUNE: Increase kP until motion is snappy. Add kD if it overshoots.
    // -----------------------------------------------------------------
    private static final double FORWARD_kP        = 1.8;  // TUNE  (m → m/s)
    private static final double FORWARD_kI        = 0.0;  // TUNE
    private static final double FORWARD_kD        = 0.06; // TUNE
    private static final double FORWARD_TOLERANCE = 0.03; // TUNE  meters

    // -----------------------------------------------------------------
    // Strafe PID  (drives robot-frame Y error → TARGET_DISTANCE_METERS)
    // Input : robot-to-target Y component  [meters]
    // Output: strafe velocity              [m/s]
    //
    // TUNE: Same process as Forward PID above.
    // -----------------------------------------------------------------
    private static final double STRAFE_kP        = 1.8;  // TUNE  (m → m/s)
    private static final double STRAFE_kI        = 0.0;  // TUNE
    private static final double STRAFE_kD        = 0.06; // TUNE
    private static final double STRAFE_TOLERANCE = 0.03; // TUNE  meters

    /**
     * Maximum allowed rotation speed (rad/s). Acts as a safety clamp so
     * the robot can't spin violently if PID gains are too high.
     *
     * TUNE: Start conservatively (0.4). Raise if alignment is too slow.
     */
    private static final double MAX_ROTATION_SPEED_RAD_S = 0.4; // TUNE

    /**
     * Maximum allowed translation speed (m/s) for both forward and strafe.
     *
     * TUNE: Start at 0.6. Raise once PID is tuned and motion looks stable.
     */
    private static final double MAX_TRANSLATION_SPEED_M_S = 0.6; // TUNE

    /**
     * Maximum accepted pose ambiguity score from PhotonVision.
     * Targets with ambiguity above this are ignored to avoid acting on
     * bad pose estimates. 0 = perfect confidence, 1 = total ambiguity.
     *
     * TUNE: If robot rejects valid targets too often, raise to 0.25.
     *       If robot acts on bad estimates, lower to 0.15.
     */
    private static final double MAX_AMBIGUITY = 0.2; // TUNE

    // =================================================================
    // ALLIANCE TAG ID SETS
    // Using HashSet for O(1) contains() lookups each execute() loop.
    // These must match VisionConstants.kRedGoalTagIDs / kBlueGoalTagIDs.
    // =================================================================

    /** Tags the robot is allowed to align to when on the RED alliance. */
    private static final Set<Integer> RED_VALID_TAG_IDS =
        new HashSet<>(Set.of(8, 5, 9, 10, 11, 2));

    /** Tags the robot is allowed to align to when on the BLUE alliance. */
    private static final Set<Integer> BLUE_VALID_TAG_IDS =
        new HashSet<>(Set.of(18, 27, 26, 25, 24, 21));

    // =================================================================
    // INSTANCE FIELDS
    // =================================================================

    /** Swerve drivetrain subsystem — required so no other command drives. */
    private final CommandSwerveDrivetrain drivetrain;

    /**
     * Vision subsystem — we read its cached PhotonPipelineResult rather
     * than creating a second PhotonCamera. Two Camera objects with the same
     * name in PhotonVision cause stale / dropped results.
     */
    private final VisionSubsystem_generated visionSubsystem;

    // PID controllers — one per alignment axis
    private final PIDController rotationPID;
    private final PIDController forwardPID;
    private final PIDController strafePID;

    /**
     * Robot-centric swerve request.
     * We use robot-centric (not field-centric) because the PID errors are
     * measured in the robot's own coordinate frame, so the drive outputs
     * must also be in that frame.
     */
    private final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric();

    /**
     * The set of valid tag IDs for this match, determined at initialize()
     * by querying the Driver Station for alliance color.
     */
    private Set<Integer> validTagIds;

    // =================================================================
    // CONSTRUCTOR
    // =================================================================

    /**
     * Creates an AlignToTagCommand.
     *
     * @param drivetrain      The swerve drivetrain subsystem.
     * @param visionSubsystem The vision subsystem that owns the PhotonCamera.
     */
    public AlignToTagCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem_generated visionSubsystem) {

        this.drivetrain      = drivetrain;
        this.visionSubsystem = visionSubsystem;

        // ------------------------------------------------------------------
        // ROTATION PID
        // enableContinuousInput wraps the yaw measurement at ±180 so the
        // controller always takes the shortest rotational path. Without this,
        // it could try to rotate 359° the wrong way.
        // ------------------------------------------------------------------
        rotationPID = new PIDController(ROTATION_kP, ROTATION_kI, ROTATION_kD);
        rotationPID.setTolerance(ROTATION_TOLERANCE);
        rotationPID.enableContinuousInput(-180.0, 180.0);

        // ------------------------------------------------------------------
        // FORWARD & STRAFE PIDs (linear, no wrapping needed)
        // ------------------------------------------------------------------
        forwardPID = new PIDController(FORWARD_kP, FORWARD_kI, FORWARD_kD);
        forwardPID.setTolerance(FORWARD_TOLERANCE);

        strafePID = new PIDController(STRAFE_kP, STRAFE_kI, STRAFE_kD);
        strafePID.setTolerance(STRAFE_TOLERANCE);

        // Declare subsystem requirements so the scheduler knows this command
        // controls the drivetrain (vision is read-only — no requirement needed).
        addRequirements(drivetrain);
    }

    // =================================================================
    // COMMAND LIFECYCLE
    // =================================================================

    /**
     * Called once when the command is first scheduled (button pressed).
     * Sets the valid tag list for this alliance and resets all PIDs.
     */
    @Override
    public void initialize() {
        // Determine alliance. Default to Blue if DS is disconnected.
        boolean isRed =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        validTagIds = isRed ? RED_VALID_TAG_IDS : BLUE_VALID_TAG_IDS;

        // Reset PIDs to clear any integral accumulation from last run.
        rotationPID.reset();
        forwardPID.reset();
        strafePID.reset();

        SmartDashboard.putString("AlignToTag/Status",   "Initializing...");
        SmartDashboard.putString("AlignToTag/Alliance", isRed ? "RED" : "BLUE");
    }

    /**
     * Called every 20ms while the button is held.
     *
     * Pipeline:
     *   1. Read latest camera result from VisionSubsystem (avoids double-consuming).
     *   2. Find the best valid alliance tag (lowest ambiguity).
     *   3. Compute robot-frame errors using the camera-to-target transform.
     *   4. Run each error through its PID controller.
     *   5. Clamp and apply robot-centric drive velocities.
     */
    @Override
    public void execute() {

        // ------------------------------------------------------------------
        // STEP 1: Get the latest camera frame from the vision subsystem.
        //         Using the subsystem's cached result avoids consuming the
        //         PhotonVision buffer twice (subsystem + command both reading).
        // ------------------------------------------------------------------
        PhotonPipelineResult latestResult = visionSubsystem.getLatestResult();

        if (latestResult == null || !latestResult.hasTargets()) {
            stopDrivetrain();
            SmartDashboard.putString("AlignToTag/Status", "No targets visible");
            return;
        }

        // ------------------------------------------------------------------
        // STEP 2: Find the best valid target in this frame.
        //         "Best" = lowest pose ambiguity among valid alliance tag IDs.
        // ------------------------------------------------------------------
        PhotonTrackedTarget bestTarget =
            selectBestTarget(latestResult.getTargets());

        if (bestTarget == null) {
            stopDrivetrain();
            SmartDashboard.putString(
                "AlignToTag/Status", "No alliance tag in view");
            return;
        }

        // ------------------------------------------------------------------
        // STEP 3: Compute robot-frame position of the target.
        //
        // getBestCameraToTarget() → 3D transform from camera lens to the tag.
        //   X = distance straight ahead of camera (= robot's left/+Y direction
        //       because camera faces left)
        //   Y = lateral offset in camera frame (= robot's forward/X direction)
        //   Z = vertical (unused for 2D alignment)
        //
        // VisionConstants.kRobotToCamera is the known camera mount transform.
        // Chaining it with camToTarget gives robotToTarget in robot frame.
        //
        // robotToTarget.getX() — forward/back relative to robot center
        //   Target is:  X > 0 if forward of center, X < 0 if behind center.
        //   GOAL: X = 0 (robot center is directly across from tag laterally).
        //
        // robotToTarget.getY() — left/right relative to robot center
        //   Target is:  Y > 0 if to the left (our camera side).
        //   GOAL: Y = TARGET_DISTANCE_METERS.
        // ------------------------------------------------------------------
        Transform3d camToTarget  = bestTarget.getBestCameraToTarget();
        Transform3d robotToTarget = VisionConstants.kRobotToCamera.plus(camToTarget);

        double robotToTargetX = robotToTarget.getTranslation().getX(); // meters
        double robotToTargetY = robotToTarget.getTranslation().getY(); // meters

        // Yaw error in degrees: positive = tag shifted to camera's right.
        // GOAL: yaw = 0 (tag centered = robot perpendicular to tag).
        double yawError = bestTarget.getYaw(); // degrees

        // ------------------------------------------------------------------
        // STEP 4: Run each error through its PID controller.
        //
        // PIDController.calculate(measurement, setpoint)
        //   returns kP*(setpoint - measurement)
        //
        // Sign analysis:
        //   forwardOutput is NEGATED before applying to velocityX because:
        //     If robotToTargetX > 0 (tag is forward): output = kP*(0-pos) < 0
        //     But we WANT to drive forward (+velocityX) → negate the output.
        //
        //   strafeOutput is NEGATED before applying to velocityY because:
        //     If robotToTargetY > TARGET (tag farther than target): output < 0
        //     But we WANT to drive left (+velocityY toward tag) → negate.
        //
        //   rotationOutput is NEGATED before applying to rotationalRate because:
        //     If yaw > 0 (tag shifted right in camera): output < 0
        //     But we WANT to rotate CCW (+rotationalRate) → negate.
        // ------------------------------------------------------------------
        double rotationOutput = clamp(
            rotationPID.calculate(yawError, 0.0),
            -MAX_ROTATION_SPEED_RAD_S,
            MAX_ROTATION_SPEED_RAD_S
        );

        double forwardOutput = clamp(
            forwardPID.calculate(robotToTargetX, 0.0),
            -MAX_TRANSLATION_SPEED_M_S,
            MAX_TRANSLATION_SPEED_M_S
        );

        double strafeOutput = clamp(
            strafePID.calculate(robotToTargetY, TARGET_DISTANCE_METERS),
            -MAX_TRANSLATION_SPEED_M_S,
            MAX_TRANSLATION_SPEED_M_S
        );

        // ------------------------------------------------------------------
        // STEP 5: Apply robot-centric swerve drive.
        //         Negate PID outputs per the sign analysis above.
        // ------------------------------------------------------------------
        drivetrain.setControl(
            robotCentricRequest
                .withVelocityX(-forwardOutput)      // forward/back
                .withVelocityY(-strafeOutput)        // left/right
                .withRotationalRate(-rotationOutput) // CCW positive
        );

        // ------------------------------------------------------------------
        // SmartDashboard telemetry — useful for PID tuning.
        // Open in Shuffleboard and add a "Number Bar" or "Graph" widget.
        // ------------------------------------------------------------------
        SmartDashboard.putNumber("AlignToTag/TargetID",         bestTarget.getFiducialId());
        SmartDashboard.putNumber("AlignToTag/YawError_deg",     yawError);
        SmartDashboard.putNumber("AlignToTag/ForwardError_m",   robotToTargetX);
        SmartDashboard.putNumber("AlignToTag/StrafeError_m",    robotToTargetY - TARGET_DISTANCE_METERS);
        SmartDashboard.putNumber("AlignToTag/PoseAmbiguity",    bestTarget.getPoseAmbiguity());
        SmartDashboard.putNumber("AlignToTag/RotationOutput",   rotationOutput);
        SmartDashboard.putNumber("AlignToTag/ForwardOutput",    forwardOutput);
        SmartDashboard.putNumber("AlignToTag/StrafeOutput",     strafeOutput);
        SmartDashboard.putBoolean("AlignToTag/IsAligned",       isAligned());
        SmartDashboard.putString("AlignToTag/Status",
            isAligned() ? "✓ ALIGNED" : "Aligning...");
    }

    /**
     * Called once when the button is released OR another command interrupts.
     * Always stops the drivetrain so the robot doesn't coast after release.
     *
     * @param interrupted true if another command interrupted us; false if
     *                    isFinished() returned true (never in our case).
     */
    @Override
    public void end(boolean interrupted) {
        stopDrivetrain();
        SmartDashboard.putString(
            "AlignToTag/Status", interrupted ? "Interrupted" : "Finished");
    }

    /**
     * This command is designed to run until the button is released, so we
     * always return false here. The whileTrue() binding in RobotContainer
     * handles cancellation on button release.
     *
     * TIP: If you want the command to automatically stop once aligned,
     * change this to:  return isAligned();
     */
    @Override
    public boolean isFinished() {
        return false; // see Javadoc above if you want auto-stop when aligned
    }

    // =================================================================
    // PRIVATE HELPER METHODS
    // =================================================================

    /**
     * Returns true when all three PID controllers are within their
     * respective tolerances simultaneously (fully aligned on all axes).
     *
     * @return true if the robot is aligned to the tag in all three DOF.
     */
    private boolean isAligned() {
        return rotationPID.atSetpoint()
            && forwardPID.atSetpoint()
            && strafePID.atSetpoint();
    }

    /**
     * Finds the best valid target from a list of detected targets.
     *
     * Selection criteria (in order):
     *   1. Tag ID must be in the current alliance's valid set.
     *   2. Pose ambiguity must be below MAX_AMBIGUITY.
     *   3. Among qualifying targets, pick the one with lowest ambiguity
     *      (most confident pose estimate = most reliable alignment).
     *
     * @param targets All targets visible in the latest camera frame.
     * @return The best valid target, or null if none qualify.
     */
    private PhotonTrackedTarget selectBestTarget(
            List<PhotonTrackedTarget> targets) {

        PhotonTrackedTarget best         = null;
        double              bestAmbiguity = Double.MAX_VALUE;

        for (PhotonTrackedTarget target : targets) {
            int    tagId     = target.getFiducialId();
            double ambiguity = target.getPoseAmbiguity();

            // Only consider valid alliance tag IDs within ambiguity threshold
            if (validTagIds.contains(tagId) && ambiguity < MAX_AMBIGUITY) {
                if (ambiguity < bestAmbiguity) {
                    bestAmbiguity = ambiguity;
                    best          = target;
                }
            }
        }
        return best; // null if no qualifying target was found
    }

    /**
     * Sends zero velocities to the drivetrain using robot-centric control.
     * Called whenever we lose sight of a target or the button is released.
     */
    private void stopDrivetrain() {
        drivetrain.setControl(
            robotCentricRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }

    /**
     * Clamps a value to the range [min, max].
     * Used to enforce maximum speed limits on PID outputs.
     *
     * @param value The raw PID output to clamp.
     * @param min   Minimum allowed value (should be negative for symmetric limits).
     * @param max   Maximum allowed value.
     * @return The clamped value.
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}