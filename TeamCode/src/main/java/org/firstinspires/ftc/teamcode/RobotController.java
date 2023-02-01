package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotController {
    private static final double     COUNTS_PER_MOTOR_REV    = 288;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double     WHEEL_DIAMETER_INCHES   = 4;
    private static final double     UNITS_TO_REV_STEPS      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public DcMotor DriveR, DriveL, LiftMotor;
    public Servo GrabberServoR, GrabberServoL;
    public BNO055IMU IMU;

    public RobotOpMode main;

    private boolean isInitialized = false;

    private double angleToMaintain = 0.0;

    private double rotation_speed = 1;
    private double speed = 1;

    public RobotController(RobotOpMode main) {
        this.main = main;
    }

    public RobotController config(String motorName, String liftMotorName, String grabberName, String imuName) {
        if (isInitialized)
            throw new RuntimeException("RobotController#config should be called before RobotController#init");

        DriveR          = main.hardwareMap.get(DcMotor.class, motorName + "_R");
        DriveL          = main.hardwareMap.get(DcMotor.class, motorName + "_L");
        LiftMotor       = main.hardwareMap.get(DcMotor.class, liftMotorName);
        GrabberServoL   = main.hardwareMap.get(Servo.class, grabberName + "_R");
        GrabberServoR   = main.hardwareMap.get(Servo.class, grabberName + "_L");
        IMU             = main.hardwareMap.get(BNO055IMU.class, imuName);

        DriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveL.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return this;
    }

    public void lift(double unit) {
        LiftMotor.setPower(1);

        LiftMotor.setTargetPosition(LiftMotor.getCurrentPosition() + (int)(unit * UNITS_TO_REV_STEPS));

        do { } while (LiftMotor.isBusy() && main.isActive());

        LiftMotor.setPower(0);
    }

    public void forward(double unit) {
        int steps = (int)(unit * UNITS_TO_REV_STEPS);

        DriveR.setPower(speed);
        DriveL.setPower(speed);

        DriveR.setTargetPosition(steps);
        DriveL.setTargetPosition(steps);

        do { } while ((DriveR.isBusy() || DriveL.isBusy()) && main.isActive());

        DriveR.setPower(0);
        DriveL.setPower(0);
    }

    public void rotate(double angle) {
        angleToMaintain = getAngle(angleToMaintain + angle);
        align();
    }

    public void align() {
        DriveR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveR.setPower(1);
        DriveL.setPower(1);

        double power, deltaAngle;
        Orientation o;

        do {
            o = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            main.telemetry.addData("M", angleToMaintain + "");
            main.telemetry.addData("A", o.thirdAngle);
            deltaAngle = getAngle(angleToMaintain - o.thirdAngle);

            power = Math.max(Math.min(deltaAngle * rotation_speed, 1), -1);

            DriveR.setPower(power);
            DriveL.setPower(-power);

            main.telemetry.addData("Delta", "" + deltaAngle);
            main.telemetry.update();
        } while (main.isActive() && Math.abs(deltaAngle) >= 0.5);

        DriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveR.setPower(0);
        DriveL.setPower(0);
    }

    private double getAngle(double angle) {
        if (angle > 180)
            return angle - 360;
        else if (angle < -180)
            return angle + 360;
        return angle;
    }

    public void init() {
        isInitialized = true;

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.mode                = BNO055IMU.SensorMode.IMU;
        param.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled      = false;

        IMU.initialize(param);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void setRotationSpeed(double speed) {
        this.rotation_speed = speed;
    }
}
