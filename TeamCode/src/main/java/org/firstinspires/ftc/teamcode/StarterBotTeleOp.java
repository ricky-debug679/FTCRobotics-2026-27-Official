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

// Match TeleOp for the StarterBot, one driver on gamepad 1. Same controls as the team's original
// StarterBot TeleOp, but driving uses the tuned Pedro drivetrain settings in Constants:
//   Left stick: drive and strafe          Right stick: turn
//   Right trigger: intake in              Left trigger: intake out
//   Hold right bumper: shoot (the windmill feeds once the launcher is up to speed)
//   D-pad up / down: launcher faster / slower
@TeleOp(name = "StarterBotTeleOp", group = "TeleOp")
public class StarterBotTeleOp extends LinearOpMode {

    // Ignore tiny trigger presses so the intake doesn't creep
    private static final double TRIGGER_DEADBAND = 0.05;

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

        telemetry.addData("Status", "Ready - press Play");
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

            // Intake: right trigger pulls in, left trigger pushes out; roller and wheels together
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(intakePower) < TRIGGER_DEADBAND) intakePower = 0;
            intakeMotor.setPower(intakePower);
            intakeServos.setPower(intakePower);

            // Launcher speed: one step faster or slower per D-pad press
            if (gamepad1.dpad_up && !previousUp) launcher.faster();
            if (gamepad1.dpad_down && !previousDown) launcher.slower();
            previousUp = gamepad1.dpad_up;
            previousDown = gamepad1.dpad_down;

            // Shooting: the launcher spins while the right bumper is held. spin() runs every loop,
            // so D-pad speed changes take effect while it is already spinning
            boolean shooting = gamepad1.right_bumper;
            if (shooting) {
                launcher.spin();
            } else {
                launcher.stop();
            }

            // Rumble once when the launcher reaches speed
            boolean ready = launcher.isReady();
            if (ready && !wasReady) gamepad1.rumble(300);
            wasReady = ready;

            // Windmill: feeds only once the launcher is fast enough to score, so no weak shots
            if (ready) {
                windmill.feed();
            } else {
                windmill.stop();
            }

            telemetry.addData("Shooter", !shooting ? "Off (hold right bumper)" : ready ? "READY - feeding" : "Spinning up...");
            telemetry.addData("Launcher speed", "%.0f / %.0f ticks/s (D-pad up/down)", launcher.getVelocity(), launcher.getTarget());
            telemetry.addData("Intake", intakePower > 0 ? "IN" : intakePower < 0 ? "OUT" : "off");
            PoseTelemetry.addPose(telemetry, localizer.pose());
            telemetry.update();
        }

        launcher.stop();
        windmill.stop();
        intakeMotor.setPower(0);
        intakeServos.setPower(0);
    }
}
