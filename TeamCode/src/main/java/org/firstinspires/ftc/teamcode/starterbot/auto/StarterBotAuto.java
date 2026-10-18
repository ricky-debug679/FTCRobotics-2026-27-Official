package org.firstinspires.ftc.teamcode.starterbot.auto;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathWatchdog;

// Leave and park for the StarterBot: one curve from a starting spot against the wall to a spot
// partly inside the alliance's loading zone, a few inches off the wall. Red1, Red2, Blue1 and Blue2
// each give only their three points; the driving is the same for all four.
public abstract class StarterBotAuto extends LinearOpMode {
    // Field positions from the Pedro visualizer: inches (0-141.5) and degrees.
    protected static final PoseFactory POSES = PoseFactory.degrees();

    // Every path takes a few seconds; much longer means the robot is stuck, so stop pushing.
    private static final double PATH_TIME_LIMIT_SECONDS = 8;

    protected abstract Pose startPose();

    protected abstract Pose controlPoint();

    protected abstract Pose parkPose();

    @Override
    public void runOpMode() {
        // Tuned drive, Pinpoint and Foresight settings shared with the TeleOp and AutoPath
        Follower follower = Constants.create(hardwareMap);
        Pose start = startPose();
        Pose park = parkPose();
        // The heading stays at the park heading the whole way, so the robot drives without turning.
        Path path = curve(start, controlPoint(), park).constant(park);

        follower.setPose(start);
        telemetry.addData("Place the robot at", "x %.1f  y %.1f  heading %.0f deg",
                start.x(), start.y(), Math.toDegrees(start.heading()));
        telemetry.addLine("Then press Start.");
        telemetry.update();

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        follower.setPose(start);
        follower.follow(path);

        // Stops the motors if the robot spins out, the Pinpoint glitches, or the path takes too long
        PathWatchdog watchdog = new PathWatchdog(PATH_TIME_LIMIT_SECONDS);
        watchdog.startPath(follower.pose());

        boolean driving = true;
        String status = "Driving to the loading zone";
        while (opModeIsActive()) {
            if (!watchdog.tripped()) {
                follower.update();
            }

            if (driving && !watchdog.check(follower.pose())) {
                driving = false;
                follower.stop();
                status = "STOPPED by safety watchdog: " + watchdog.reason();
            } else if (driving && follower.atParametricEnd()) {
                driving = false;
                follower.hold(park);
                status = "Parked";
            }

            Pose pose = follower.pose();
            telemetry.addData("Status", status);
            telemetry.addData("Time (s)", "%.1f", timer.seconds());
            if (driving) {
                telemetry.addData("Distance left (in)", "%.1f", follower.distanceToEndpoint());
            }
            telemetry.addData("x", "%.1f", pose.x());
            telemetry.addData("y", "%.1f", pose.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.update();
        }
    }
}
