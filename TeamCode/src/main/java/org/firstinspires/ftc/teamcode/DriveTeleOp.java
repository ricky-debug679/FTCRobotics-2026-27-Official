package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseTelemetry;

@TeleOp(name = "DriveTeleOp", group = "TeleOp")
public class DriveTeleOp extends LinearOpMode {

    private static final double SLOW_MODE_SCALE = 0.4;

    @Override
    public void runOpMode() {
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        telemetry.addData("Status", "Ready - press Play");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            // Left stick = drive/strafe, right stick X = turn
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // Hold right bumper for slow, precise driving
            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
            drivetrain.drive(new DrivePowers(forward * scale, strafe * scale, turn * scale), true);

            telemetry.addData("Status", gamepad1.right_bumper ? "Driving (SLOW)" : "Driving");
            telemetry.addData("forward / strafe / turn", "%.2f / %.2f / %.2f", forward, strafe, turn);
            PoseTelemetry.addPose(telemetry, localizer.pose());
            telemetry.update();
        }
    }
}
