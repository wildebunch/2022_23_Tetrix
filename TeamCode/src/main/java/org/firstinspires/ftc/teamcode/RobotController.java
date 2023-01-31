package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotController {
    public DcMotor DriveR, DriveL, LiftMotor;
    public Servo GrabberServoR, GrabberServoL;

    public RobotOpMode main;

    private boolean isInitialized;

    public RobotController(RobotOpMode main) {
        this.main = main;
    }

    public RobotController config(String motorName, String liftMotorName, String grabberName) {
        if (isInitialized)
            throw new Exception("RobotController#config should be called befor ");

        DriveR          = main.hardwareMap.get(DcMotor.class, motorName + "_R");
        DriveL          = main.hardwareMap.get(DcMotor.class, motorName + "_L");
        LiftMotor       = main.hardwareMap.get(DcMotor.class, liftMotorName);
        GrabberServoL   = main.hardwareMap.get(Servo.class, grabberName + "_R");
        GrabberServoR   = main.hardwareMap.get(Servo.class, grabberName + "_L");

        return this;
    }
}
