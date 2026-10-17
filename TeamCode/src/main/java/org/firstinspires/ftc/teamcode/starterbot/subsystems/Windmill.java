package org.firstinspires.ftc.teamcode.starterbot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// The windmill servo (a goBILDA Torque servo) that pushes pollen into the launcher.
public class Windmill {
    private final CRServo servo;

    public Windmill(HardwareMap hardwareMap) {
        servo = GoBildaServo.get(hardwareMap, "windmillServo");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        stop();
    }

    public void feed() {
        servo.setPower(1);
    }

    public void stop() {
        servo.setPower(0);
    }
}
