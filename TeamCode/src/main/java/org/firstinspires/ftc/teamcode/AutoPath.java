package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathWatchdog;
import org.firstinspires.ftc.teamcode.pedro.PoseTelemetry;

@Autonomous(name = "AutoPath", group = "Autonomous", preselectTeleOp = "DriveTeleOp")
public class AutoPath extends LinearOpMode {

    private static final int TOTAL_PATHS = 5;
    // The longest path (point3 -> point4, ~110 in) takes a few seconds; longer than this means something is wrong
    private static final double WATCHDOG_MAX_PATH_SECONDS = 8;

    private Follower follower;
    private final ElapsedTime runtime = new ElapsedTime();

    // Updated by the routine's commands so the Driver Hub shows what the robot is doing
    private String status = "Initializing";
    private int currentStep = 0;
    private boolean finished = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose point1 = poseFactory.of(56.3187, 36.9561, 90);
    private final Pose point2 = poseFactory.of(12.9651, 23.2849, -162.4978);
    private final Pose point3 = poseFactory.of(5.2714, 3.634, -111.3813);
    private final Pose point4 = poseFactory.of(57.2185, 102.2883, 270);
    private final Pose point5 = poseFactory.of(6.1734, 96.5495, 6.4145);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                step(1, "start -> point1", path1()),
                pause(2500),
                step(2, "point1 -> point2", path2()),
                step(3, "point2 -> point3", path3()),
                pause(2500),
                step(4, "point3 -> point4", path4()),
                pause(2500),
                step(5, "point4 -> point5", path5()),
                instant(() -> {
                    finished = true;
                    status = "Auto complete";
                })
        );
    }

    // Follows a path, first recording which step is running for the telemetry display
    private Command step(int number, String description, Path path) {
        return sequential(
                instant(() -> {
                    currentStep = number;
                    status = "Path " + number + "/" + TOTAL_PATHS + ": " + description;
                }),
                follow(follower, path)
        );
    }

    // Waits in place at the end of the previous path; follower.update() keeps holding the pose
    private Command pause(double milliseconds) {
        return sequential(
                instant(() -> status = "Waiting " + (int) milliseconds + " ms after path " + currentStep),
                waitMs(milliseconds)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        telemetry.addData("Status", "Creating follower (Pinpoint + mecanum)...");
        telemetry.update();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Show live localizer readings during init so you can check the start pose before Play.
        // Only the localizer is updated here, so the motors stay off.
        while (opModeInInit()) {
            follower.localizer.update();
            telemetry.addData("Status", "Ready - press Play");
            telemetry.addData("Expected start", "x=%.1f  y=%.1f  heading=%.1f deg",
                    start.x(), start.y(), Math.toDegrees(start.heading()));
            PoseTelemetry.addPose(telemetry, follower.localizer.pose());
            telemetry.update();
        }

        waitForStart();
        runtime.reset();
        status = "Starting";
        schedule(autoRoutine());

        PathWatchdog watchdog = new PathWatchdog(WATCHDOG_MAX_PATH_SECONDS);
        Path watchedPath = null;

        while (opModeIsActive()) {
            if (watchdog.tripped()) {
                telemetry.addData("Status", "STOPPED by safety watchdog: " + watchdog.reason());
                PoseTelemetry.addPose(telemetry, follower.pose());
                telemetry.update();
                continue;
            }

            follower.update();

            // Only watch while a path is being followed (not during the waits at the end of a path)
            Path currentPath = follower.following() ? follower.currentPath() : null;
            if (currentPath != null) {
                if (currentPath != watchedPath) {
                    watchedPath = currentPath;
                    watchdog.startPath(follower.pose());
                }
                if (!watchdog.check(follower.pose())) {
                    follower.stop();
                    continue;
                }
            } else {
                watchedPath = null;
            }

            Scheduler.execute();

            telemetry.addData("Status", status);
            telemetry.addData("Time (s)", "%.1f", runtime.seconds());
            PoseTelemetry.addPose(telemetry, follower.pose());

            if (!finished && follower.currentPath() != null) {
                telemetry.addData("Path", "%d / %d", currentStep, TOTAL_PATHS);
                telemetry.addData("Distance to end (in)", "%.2f", follower.distanceToEndpoint());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(start, point1).constant(point1);
    }

    public Path path2() {
        return line(point1, point2).tangent();
    }

    public Path path3() {
        return line(point2, point3).tangent();
    }

    public Path path4() {
        return line(point3, point4).constant(point4);
    }

    public Path path5() {
        return line(point4, point5).reverseTangent();
    }
}