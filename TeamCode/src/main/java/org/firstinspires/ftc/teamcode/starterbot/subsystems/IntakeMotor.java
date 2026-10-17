package org.firstinspires.ftc.teamcode.starterbot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// The intake roller motor.
public class IntakeMotor {
    private final DcMotor motor;

    public IntakeMotor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake");
        // Reversed: on the first test, forward power spun the intake outward.
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Positive pulls pollen in, negative pushes it out, -1 to 1.
    public void setPower(double power) {
        motor.setPower(power);
    }
}
