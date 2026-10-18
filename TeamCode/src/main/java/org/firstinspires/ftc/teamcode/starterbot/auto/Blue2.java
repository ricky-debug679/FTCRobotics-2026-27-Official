package org.firstinspires.ftc.teamcode.starterbot.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Bottom wall near the right corner, facing up the field. Rises straight into the bottom of the
// blue loading zone.
@SuppressWarnings("unused") // the FTC app finds this through @Autonomous
@Autonomous(name = "StarterBot Blue 2", group = "StarterBot", preselectTeleOp = "StarterBotTeleOp")
public class Blue2 extends StarterBotAuto {
    @Override
    protected Pose startPose() {
        return POSES.of(121.5, 8.5, 90);
    }

    @Override
    protected Pose controlPoint() {
        return POSES.of(121.5, 17.5, 0);
    }

    @Override
    protected Pose parkPose() {
        return POSES.of(129.5, 25.5, 90);
    }
}
