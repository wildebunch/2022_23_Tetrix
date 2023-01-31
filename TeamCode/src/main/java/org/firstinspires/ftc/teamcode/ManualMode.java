package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Manual", group="")
public class ManualMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor RightMotor, LeftMotor;
        RightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        LeftMotor = hardwareMap.get(DcMotor.class, "left_motor");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Mode", "Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Right Wheel Power", "" + Math.min(Math.max(gamepad1.left_stick_y - gamepad1.left_stick_x, -1), 1));
            telemetry.addData("Left Wheel Power", "" + Math.min(Math.max(gamepad1.left_stick_y + gamepad1.left_stick_x, -1), 1));
            telemetry.update();

            RightMotor.setPower(Math.min(Math.max(gamepad1.left_stick_y - gamepad1.left_stick_x, -1), 1));
            LeftMotor.setPower(Math.min(Math.max(gamepad1.left_stick_y + gamepad1.left_stick_x, -1), 1));
        }
    }
}
