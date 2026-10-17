package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// Stage 2: drive normally with the sticks. While the LEFT BUMPER is held, the robot turns in
// place to face the nearest POLLEN seen by the Limelight. Let go to take control back.
@TeleOp(name = "PollenTrackTeleOp", group = "TeleOp")
public class PollenTrackTeleOp extends LinearOpMode {

    // Pipeline 0 = "pollen" (yellow color pipeline set up in the Limelight web page)
    private static final int POLLEN_PIPELINE = 0;

    // Measured during the Limelight tests: ta ≈ 10.3% where the robot should stop, ≈ 0.24% far away
    private static final double STOP_AREA = 10.3;
    // Ball counts as "centered" when tx is within this many degrees
    private static final double CENTERED_DEGREES = 3.0;

    // Turn power per degree the ball is off-center, and the most turn power tracking may use
    private static final double TURN_GAIN = 0.02;
    private static final double MAX_TURN = 0.4;

    // Smoothing: the ball must be seen this long before tracking starts (ignores one-frame false hits),
    // and must be missing this long before tracking stops (ignores one-frame misses)
    private static final double CONFIRM_MS = 60;
    private static final double LOSE_MS = 150;

    private boolean ballConfirmed = false;
    private double lastTx = 0;
    private double lastTy = 0;
    private double lastTa = 0;
    private final ElapsedTime seenTimer = new ElapsedTime();
    private final ElapsedTime missingTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        // "limelight" must match the name in the Driver Station robot configuration
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(POLLEN_PIPELINE);
        limelight.start();

        // Show the camera data during INIT too; the motors stay off until Play
        while (opModeInInit()) {
            updateVision(limelight);
            telemetry.addData("Status", "Ready - press Play");
            addVisionTelemetry(limelight);
            telemetry.update();
        }

        while (opModeIsActive()) {
            updateVision(limelight);

            // Left stick = drive/strafe, right stick X = turn (same as DriveTeleOp)
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            String status = "Driving (hold LEFT BUMPER to face POLLEN)";

            if (gamepad1.left_bumper) {
                // Stage 2 only turns in place, so the sticks' drive and strafe are ignored
                forward = 0;
                strafe = 0;
                if (ballConfirmed) {
                    // Pedro turn is + counterclockwise (left), so a ball on the right (tx +) needs a negative turn
                    turn = Range.clip(-TURN_GAIN * lastTx, -MAX_TURN, MAX_TURN);
                    status = Math.abs(lastTx) <= CENTERED_DEGREES ? "Tracking: facing POLLEN" : "Tracking: turning to POLLEN";
                } else {
                    turn = 0;
                    status = "Tracking: no POLLEN in view";
                }
            }

            drivetrain.drive(new DrivePowers(forward, strafe, turn), true);

            telemetry.addData("Status", status);
            telemetry.addData("forward / strafe / turn", "%.2f / %.2f / %.2f", forward, strafe, turn);
            addVisionTelemetry(limelight);
            telemetry.update();
        }

        drivetrain.stop();
        limelight.stop();
    }

    // Reads the newest Limelight result and applies the seen/missing smoothing
    private void updateVision(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        boolean seenNow = result != null && result.isValid();

        if (seenNow) {
            lastTx = result.getTx();   // + = ball is to the right, - = left
            lastTy = result.getTy();
            lastTa = result.getTa();   // % of the image the ball covers (bigger = closer)
            missingTimer.reset();
            if (!ballConfirmed && seenTimer.milliseconds() >= CONFIRM_MS) {
                ballConfirmed = true;
            }
        } else {
            seenTimer.reset();
            if (ballConfirmed && missingTimer.milliseconds() >= LOSE_MS) {
                ballConfirmed = false;
            }
        }
    }

    private void addVisionTelemetry(Limelight3A limelight) {
        telemetry.addData("Limelight", limelight.isConnected() ? "connected" : "NOT CONNECTED");

        if (ballConfirmed) {
            String side = Math.abs(lastTx) <= CENTERED_DEGREES ? "CENTERED" : (lastTx > 0 ? "RIGHT" : "LEFT");
            String distance = lastTa >= STOP_AREA ? "AT STOP DISTANCE" : "approaching";

            telemetry.addData("POLLEN", "SEEN");
            telemetry.addData("tx / ty / ta", "%.2f° / %.2f° / %.3f%%", lastTx, lastTy, lastTa);
            telemetry.addData("Ball is", "%s, %s", side, distance);
        } else {
            telemetry.addData("POLLEN", "not seen");
        }
    }
}
