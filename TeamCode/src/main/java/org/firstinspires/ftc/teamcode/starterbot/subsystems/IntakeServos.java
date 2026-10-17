package org.firstinspires.ftc.teamcode.starterbot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// The left and right intake wheel servos (goBILDA Speed servos). They always spin together.
public class IntakeServos {
    private final CRServo left;
    private final CRServo right;

    public IntakeServos(HardwareMap hardwareMap) {
        left = GoBildaServo.get(hardwareMap, "left_intake_servo");
        right = GoBildaServo.get(hardwareMap, "right_intake_servo");
        // The right servo is reversed so both pull pollen inward.
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        setPower(0);
    }

    // Positive pulls pollen in, negative pushes it out, -1 to 1.
    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }
}
