package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotController {
    public DcMotor DriveR, DriveL, LiftMotor;
    public Servo GrabberServoR, GrabberServoL;
    public BNO055IMU IMU;

    public RobotOpMode main;

    private boolean isInitialized = false;

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

        return this;
    }

    public RobotController init() {
        isInitialized = true;

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.mode                = BNO055IMU.SensorMode.IMU;
        param.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled      = false;

        IMU.initialize(param);

        return this;
    }
}
