package org.firstinspires.ftc.teamcode.starterbot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

// goBILDA Speed and Torque servos take 500-2500 microsecond pulses; the SDK's default 600-2400
// stops short of full speed. Every StarterBot servo, and Servo Test, is set up with this range.
public final class GoBildaServo {
    public static final PwmControl.PwmRange RANGE = new PwmControl.PwmRange(500, 2500);

    private GoBildaServo() {
    }

    public static CRServo get(HardwareMap hardwareMap, String name) {
        CRServoImplEx servo = hardwareMap.get(CRServoImplEx.class, name);
        servo.setPwmRange(RANGE);
        return servo;
    }
}
