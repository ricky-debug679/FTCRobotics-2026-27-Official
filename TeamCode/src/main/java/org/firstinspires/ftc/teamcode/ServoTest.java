package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.starterbot.subsystems.GoBildaServo;

// Tests the StarterBot's continuous rotation servos one at a time, and explains on screen why
// one can't be found (wrong name, or configured as "Servo" instead of "Continuous Rotation Servo").
//   D-pad left / right: choose the servo
//   Left stick up / down: spin it forward / backward (proportional)
//   Hold A: full forward     Hold B: full backward
// Only the chosen servo moves; the others are held at 0. Directions here are the raw servo
// directions (the TeleOp reverses some of them), so "forward" may look backward on the robot.
@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends LinearOpMode {

    private static final String[] NAMES = {"left_intake_servo", "right_intake_servo", "windmillServo"};

    @Override
    public void runOpMode() {
        CRServo[] servos = new CRServo[NAMES.length];
        String[] problems = new String[NAMES.length];

        for (int i = 0; i < NAMES.length; i++) {
            try {
                servos[i] = GoBildaServo.get(hardwareMap, NAMES[i]);
                servos[i].setPower(0);
            } catch (Exception e) {
                problems[i] = diagnose(NAMES[i]);
            }
        }

        int selected = 0;
        boolean previousLeft = false, previousRight = false;

        while (opModeInInit()) {
            telemetry.addData("Status", "Ready - press Play");
            addServoList(servos, problems, selected);
            telemetry.update();
        }

        while (opModeIsActive()) {
            if (gamepad1.dpad_right && !previousRight) selected = (selected + 1) % NAMES.length;
            if (gamepad1.dpad_left && !previousLeft) selected = (selected + NAMES.length - 1) % NAMES.length;
            previousRight = gamepad1.dpad_right;
            previousLeft = gamepad1.dpad_left;

            double power = -gamepad1.left_stick_y;
            if (gamepad1.a) power = 1;
            if (gamepad1.b) power = -1;
            if (Math.abs(power) < 0.05) power = 0;

            for (int i = 0; i < servos.length; i++) {
                if (servos[i] != null) servos[i].setPower(i == selected ? power : 0);
            }

            telemetry.addData("Controls", "D-pad L/R = choose, left stick / A / B = spin");
            telemetry.addData("Power to " + NAMES[selected], "%.2f", power);
            addServoList(servos, problems, selected);
            telemetry.update();
        }

        for (CRServo servo : servos) {
            if (servo != null) servo.setPower(0);
        }
    }

    private void addServoList(CRServo[] servos, String[] problems, int selected) {
        for (int i = 0; i < NAMES.length; i++) {
            String marker = i == selected ? ">> " : "   ";
            if (servos[i] != null) {
                telemetry.addData(marker + NAMES[i], "found, port %d", servos[i].getPortNumber());
            } else {
                telemetry.addData(marker + NAMES[i], problems[i]);
            }
        }
    }

    // Works out why a continuous rotation servo with this name couldn't be found
    private String diagnose(String name) {
        try {
            hardwareMap.get(Servo.class, name);
            return "WRONG TYPE: configured as \"Servo\" - change it to \"Continuous Rotation Servo\"";
        } catch (Exception notAServo) {
            return "NOT FOUND: no device named \"" + name + "\" in the active configuration";
        }
    }
}
