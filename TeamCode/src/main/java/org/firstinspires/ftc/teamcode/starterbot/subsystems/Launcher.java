package org.firstinspires.ftc.teamcode.starterbot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// The shooter wheel. Its speed is a fraction of full speed (0.5 = 50%), changed by the driver with
// the D-pad. It can run two ways:
//   USE_ENCODER = false: plain motor power. Works without the encoder; "ready" is a fixed spin-up time.
//   USE_ENCODER = true:  holds the speed using the encoder (steadier shots as the battery drains).
//                        The encoder cable must be plugged in beside the motor port, or the wheel
//                        runs at full power because it always reads 0.
public class Launcher {
    public static final boolean USE_ENCODER = false;

    // A 6000 RPM goBILDA motor with a 28 tick/turn encoder tops out near 2800 ticks/s
    private static final double MAX_TICKS_PER_SECOND = 2800;
    // Speed as a fraction of full speed. It starts here every run; the D-pad changes it in steps.
    // 50% landed a couple of inches short most of the time in driver testing, so start a little higher
    private static final double START_SPEED = 0.53;
    private static final double STEP = 0.02;
    private static final double LOWEST = 0.20;
    private static final double HIGHEST = 1.0;
    // Without the encoder: ready this long after starting (or changing speed)
    private static final double SPIN_UP_SECONDS = 1.5;
    // With the encoder: ready once within this many ticks/s of the target, so shots leave at full speed
    private static final double TOLERANCE = 50;
    // From goBILDA's example: tuned for a launcher wheel, not a drive wheel.
    private static final PIDFCoefficients PIDF = new PIDFCoefficients(40, 0, 0, 12.5);

    private final DcMotorEx motor;
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private double speed = START_SPEED;
    private boolean spinning;

    public Launcher(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        // Let the heavy wheel coast down instead of braking it hard
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (USE_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void faster() {
        speed = Range.clip(speed + STEP, LOWEST, HIGHEST);
        spinUpTimer.reset();
    }

    public void slower() {
        speed = Range.clip(speed - STEP, LOWEST, HIGHEST);
        spinUpTimer.reset();
    }

    // Call every loop while it should spin, so speed changes apply right away
    public void spin() {
        if (!spinning) spinUpTimer.reset();
        spinning = true;
        if (USE_ENCODER) {
            motor.setVelocity(speed * MAX_TICKS_PER_SECOND);
        } else {
            motor.setPower(speed);
        }
    }

    public void stop() {
        spinning = false;
        motor.setPower(0);
    }

    // True while spinning and fast enough to score
    public boolean isReady() {
        if (!spinning) return false;
        if (USE_ENCODER) return getVelocity() > speed * MAX_TICKS_PER_SECOND - TOLERANCE;
        return spinUpTimer.seconds() >= SPIN_UP_SECONDS;
    }

    // Fraction of full speed, e.g. 0.5 = 50%
    public double getSpeed() {
        return speed;
    }

    // Measured speed from the encoder (reads 0 if the encoder cable isn't plugged in)
    public double getVelocity() {
        return motor.getVelocity();
    }
}
