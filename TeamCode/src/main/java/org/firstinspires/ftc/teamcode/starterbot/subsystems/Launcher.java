package org.firstinspires.ftc.teamcode.starterbot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

// The shooter wheel. It holds a speed instead of a power, using its encoder, which must be
// plugged in beside its motor port. Speeds are in encoder ticks per second (28 per motor turn).
public class Launcher {
    // 1250 is about 2680 RPM. The driver changes the target with the D-pad; it starts here every run.
    private static final double START_VELOCITY = 1250;
    private static final double STEP = 50;
    private static final double LOWEST = 500;
    private static final double HIGHEST = 2500;
    // Ready once within this much of the target, so shots leave at full speed.
    private static final double TOLERANCE = 50;
    // From goBILDA's example: tuned for a launcher wheel, not a drive wheel.
    private static final PIDFCoefficients PIDF = new PIDFCoefficients(40, 0, 0, 12.5);

    private final DcMotorEx motor;
    private double target = START_VELOCITY;
    private boolean spinning;

    public Launcher(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
    }

    public void faster() {
        target = Range.clip(target + STEP, LOWEST, HIGHEST);
    }

    public void slower() {
        target = Range.clip(target - STEP, LOWEST, HIGHEST);
    }

    public void spin() {
        spinning = true;
        motor.setVelocity(target);
    }

    public void stop() {
        spinning = false;
        motor.setVelocity(0);
    }

    // True while spinning and fast enough to score.
    public boolean isReady() {
        return spinning && getVelocity() > target - TOLERANCE;
    }

    public double getTarget() {
        return target;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }
}
