package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Autonomous", group="")
public class AutoMode extends RobotOpMode {
    @Override
    public void runOpMode() {
        RobotController controller = new RobotController(this)
                .config("driver", "lift", "grabber", "imu");

        telemetry.addData("Mode", "Waiting for start...");
        telemetry.update();
        waitForStart();

        controller.init();

        while (isActive()) {

        }
    }
}
