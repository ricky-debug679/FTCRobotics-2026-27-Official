package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;

// Safety check for path following: trips when the robot spins far more than any path needs,
// when the pose jumps further than the robot can physically move in one loop (a localizer
// glitch), or when a single path takes far too long (the robot never "arrives").
// Call startPath() when a new path begins and check() every loop; stop the motors when it trips.
public class PathWatchdog {

    // No path here turns more than about half a turn, so 1.5 full turns means the robot is spinning out
    private static final double MAX_ROTATION_RADIANS = 3 * Math.PI;
    // At ~80 in/s and ~20 ms loops the robot moves under 2 in per loop
    private static final double MAX_JUMP_INCHES = 12;
    private static final double MAX_JUMP_RADIANS = Math.PI / 2;

    private final double maxPathSeconds;
    private final ElapsedTime pathTimer = new ElapsedTime();
    private Pose lastPose;
    private double rotation;
    private String tripReason = null;

    public PathWatchdog(double maxPathSeconds) {
        this.maxPathSeconds = maxPathSeconds;
    }

    public void startPath(Pose currentPose) {
        lastPose = currentPose;
        rotation = 0;
        pathTimer.reset();
    }

    // Returns false (and stays false) once something unsafe has happened
    public boolean check(Pose currentPose) {
        if (tripped()) return false;
        if (lastPose == null) {
            startPath(currentPose);
            return true;
        }

        double moved = currentPose.distance(lastPose);
        double turned = Math.abs(Angle.normalizeSigned(currentPose.heading() - lastPose.heading()));
        lastPose = currentPose;
        rotation += turned;

        if (Double.isNaN(currentPose.x()) || Double.isNaN(currentPose.y()) || Double.isNaN(currentPose.heading())) {
            tripReason = "position reading is NaN (Pinpoint glitch)";
        } else if (moved > MAX_JUMP_INCHES || turned > MAX_JUMP_RADIANS) {
            tripReason = String.format("position jumped %.1f in / %.0f deg in one loop (Pinpoint glitch)",
                    moved, Math.toDegrees(turned));
        } else if (rotation > MAX_ROTATION_RADIANS) {
            tripReason = String.format("spun %.0f deg during one path (spinning out)", Math.toDegrees(rotation));
        } else if (pathTimer.seconds() > maxPathSeconds) {
            tripReason = String.format("one path took over %.0f s (never arrived)", maxPathSeconds);
        }
        return !tripped();
    }

    public boolean tripped() {
        return tripReason != null;
    }

    public String reason() {
        return tripReason;
    }
}
