package org.firstinspires.ftc.teamcode.starterbot.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Bottom wall, facing up the field. Curves left of the Hive into the bottom of the red loading zone.
@SuppressWarnings("unused") // the FTC app finds this through @Autonomous
@Autonomous(name = "StarterBot Red 1", group = "StarterBot", preselectTeleOp = "StarterBotTeleOp")
public class Red1 extends StarterBotAuto {
    @Override
    protected Pose startPose() {
        return POSES.of(56, 8.5, 90);
    }

    @Override
    protected Pose controlPoint() {
        return POSES.of(12.38, 43.04, 0);
    }

    @Override
    protected Pose parkPose() {
        return POSES.of(12, 96, 90);
    }
}
