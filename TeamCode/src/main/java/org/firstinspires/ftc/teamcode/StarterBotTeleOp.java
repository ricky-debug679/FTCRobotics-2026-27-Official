package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.starterbot.subsystems.IntakeServos;
import org.firstinspires.ftc.teamcode.starterbot.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.starterbot.subsystems.Windmill;

// Match TeleOp for the StarterBot, one driver on gamepad 1. Driving uses the tuned Pedro
// drivetrain settings in Constants. The intake and launcher run the whole match:
//   Left stick: drive and strafe          Right stick: turn
//   Intake always pulls pollen in (it never reverses)
//   Launcher spins all the time           Hold right bumper: shoot (feeds once the launcher is ready)
//   D-pad up / down: launcher speed +/- 2%
//   Hold Y (triangle): feed now, without waiting for the launcher to be ready
@TeleOp(name = "StarterBotTeleOp", group = "TeleOp")
public class StarterBotTeleOp extends LinearOpMode {

    // The intake pulls in at this power the whole match; nothing reverses it
    private static final double INTAKE_RUN_POWER = 1.0;

    @Override
    public void runOpMode() {
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Windmill windmill = new Windmill(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);

        boolean wasReady = false;
        boolean previousUp = false, previousDown = false;

        telemetry.addData("Status", "Ready - press Play (intake and launcher start right away)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            // Driving (same as DriveTeleOp). Pedro's strafe is + left and turn is + counterclockwise,
            // so pushing a stick right (positive x) needs a negative value
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            drivetrain.drive(new DrivePowers(forward, strafe, turn), true);

            // Intake: always pulling pollen in, roller and wheels together. Set every loop so it
            // keeps running in the same direction no matter what else is happening
            intakeMotor.setPower(INTAKE_RUN_POWER);
            intakeServos.setPower(INTAKE_RUN_POWER);

            // Launcher speed: one step faster or slower per D-pad press
            if (gamepad1.dpad_up && !previousUp) launcher.faster();
            if (gamepad1.dpad_down && !previousDown) launcher.slower();
            previousUp = gamepad1.dpad_up;
            previousDown = gamepad1.dpad_down;

            // The launcher spins the whole match; called every loop so speed changes apply right away
            launcher.spin();

            // Rumble once when the launcher is ready
            boolean ready = launcher.isReady();
            if (ready && !wasReady) gamepad1.rumble(300);
            wasReady = ready;

            // Windmill: hold the right bumper to shoot once the launcher is ready, so no weak shots.
            // Y feeds right away without waiting
            boolean shoot = gamepad1.right_bumper;
            boolean manualFeed = gamepad1.y;
            String feedStatus;
            if (manualFeed) {
                windmill.feed();
                feedStatus = "FEEDING (Y, manual)";
            } else if (shoot && ready) {
                windmill.feed();
                feedStatus = "FEEDING";
            } else {
                windmill.stop();
                feedStatus = shoot ? "waiting for launcher..." : "off (hold right bumper to shoot)";
            }

            telemetry.addData("Launcher", "%.0f%% speed - %s (D-pad up/down)",
                    launcher.getSpeed() * 100, ready ? "READY" : "spinning up...");
            telemetry.addData("Launcher encoder", Launcher.USE_ENCODER
                    ? String.format("%.0f ticks/s", launcher.getVelocity())
                    : String.format("%.0f ticks/s (not used for control)", launcher.getVelocity()));
            telemetry.addData("Feed", feedStatus);
            telemetry.addData("Intake", "IN (always)");
            PoseTelemetry.addPose(telemetry, localizer.pose());
            telemetry.update();
        }

        launcher.stop();
        windmill.stop();
        intakeMotor.setPower(0);
        intakeServos.setPower(0);
    }
}
