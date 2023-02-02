package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Manual", group="")
public class ManualMode extends LinearOpMode {

    DcMotor RightMotor, LeftMotor, LiftMotor;
    Servo LeftGrabber, RightGrabber;

    @Override
    public void runOpMode() {
        RightMotor = hardwareMap.get(DcMotor.class, "drive_R");
        LeftMotor = hardwareMap.get(DcMotor.class, "drive_L");
        LiftMotor = hardwareMap.get(DcMotor.class, "lift");
        LeftGrabber = hardwareMap.get(Servo.class, "grabber_R");
        RightGrabber = hardwareMap.get(Servo.class, "grabber_L");

        RightMotor. setDirection(DcMotorSimple.Direction.REVERSE);

        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Mode", "Waiting for start...");
        telemetry.update();
        waitForStart();

        double rightPower, leftPower, liftPower;
        while (opModeIsActive()) {
            rightPower = Math.min(Math.max(gamepad1.left_stick_y + gamepad1.left_stick_x, -1), 1);
            leftPower = Math.min(Math.max(gamepad1.left_stick_y - gamepad1.left_stick_x, -1), 1);
            liftPower = Math.min(Math.max(gamepad1.right_stick_y, -1), 1);

            telemetry.addData("Right Wheel Power", "" + rightPower);
            telemetry.addData("Left Wheel Power", "" + leftPower);
            telemetry.addData("Lift Power", "" + liftPower);
            telemetry.update();

            RightMotor.setPower(rightPower);
            LeftMotor.setPower(leftPower);
            LiftMotor.setPower(liftPower);

            setGrabberPosition(LeftGrabber.getPosition() + (gamepad1.left_trigger - gamepad1.right_trigger) * 0.1);
        }
    }

    public void setGrabberPosition(double grabberPosition) {
        LeftGrabber.setPosition(Math.min(Math.max(grabberPosition, -1), 1));
        RightGrabber.setPosition(1 - Math.min(Math.max(grabberPosition, -1), 1));
    }
}
