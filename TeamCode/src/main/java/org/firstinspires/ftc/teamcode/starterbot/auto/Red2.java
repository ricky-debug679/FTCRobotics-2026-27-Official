package org.firstinspires.ftc.teamcode.starterbot.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Top wall near the left corner, facing down the field. Drops straight into the top of the red
// loading zone.
@SuppressWarnings("unused") // the FTC app finds this through @Autonomous
@Autonomous(name = "StarterBot Red 2", group = "StarterBot", preselectTeleOp = "StarterBotTeleOp")
public class Red2 extends StarterBotAuto {
    @Override
    protected Pose startPose() {
        return POSES.of(20, 133, 270);
    }

    @Override
    protected Pose controlPoint() {
        return POSES.of(20, 124, 0);
    }

    @Override
    protected Pose parkPose() {
        return POSES.of(11, 115, 270);
    }
}
