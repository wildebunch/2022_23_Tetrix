package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="2022-23 IMU Test", group="")
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor FLMotor, FRMotor, BLMotor, BRMotor, LiftMotor;
        BNO055IMU imu;

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.mode = BNO055IMU.SensorMode.IMU;
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(param);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        double power, deltaAngle;
        Orientation maintainAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        Acceleration a;
        while (opModeIsActive()) {
            Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            a = imu.getLinearAcceleration();
            deltaAngle = maintainAngle.thirdAngle - o.thirdAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            power = Math.max(Math.min(deltaAngle / 25, 1), -1);

            telemetry.addLine("==== Angle ====");
            telemetry.addData("Delta", "" + (deltaAngle));
            telemetry.addData("X", "" + o.secondAngle);
            telemetry.addData("Y", "" + o.firstAngle);
            telemetry.addData("Z", "" + o.thirdAngle);
            telemetry.addLine("==== Acceleration ====");
            telemetry.addData("X", "" + a.xAccel);
            telemetry.addData("Y", "" + a.yAccel);
            telemetry.addData("Z", "" + a.zAccel);
            telemetry.update();
            idle();
        }
    }
}
