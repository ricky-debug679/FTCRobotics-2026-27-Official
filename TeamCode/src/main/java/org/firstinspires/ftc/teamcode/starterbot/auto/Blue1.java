package org.firstinspires.ftc.teamcode.starterbot.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Top wall, facing down the field. Curves right of the Hive into the top of the blue loading zone.
@SuppressWarnings("unused") // the FTC app finds this through @Autonomous
@Autonomous(name = "StarterBot Blue 1", group = "StarterBot", preselectTeleOp = "StarterBotTeleOp")
public class Blue1 extends StarterBotAuto {
    @Override
    protected Pose startPose() {
        return POSES.of(85.5, 133, 270);
    }

    @Override
    protected Pose controlPoint() {
        return POSES.of(129.12, 98.46, 0);
    }

    @Override
    protected Pose parkPose() {
        return POSES.of(129.5, 45.5, 270);
    }
}
