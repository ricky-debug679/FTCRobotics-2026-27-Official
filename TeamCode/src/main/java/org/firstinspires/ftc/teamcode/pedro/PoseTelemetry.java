package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Shared Driver Hub readout so every OpMode shows the robot's position the same way.
public final class PoseTelemetry {

    private PoseTelemetry() {
    }

    public static void addPose(Telemetry telemetry, Pose pose) {
        telemetry.addData("x (in)", "%.2f", pose.x());
        telemetry.addData("y (in)", "%.2f", pose.y());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
    }

    public static void show(Telemetry telemetry, String status, Pose pose) {
        telemetry.addData("Status", status);
        addPose(telemetry, pose);
        telemetry.update();
    }
}
