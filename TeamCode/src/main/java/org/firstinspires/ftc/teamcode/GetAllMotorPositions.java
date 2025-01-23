package org.firstinspires.ftc.teamcode;

import android.view.animation.LinearInterpolator;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class GetAllMotorPositions extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft0 = hardwareMap.dcMotor.get("frontLeft0");
        DcMotor frontRight1 = hardwareMap.dcMotor.get("frontRight1");
        DcMotor backLeft2 = hardwareMap.dcMotor.get("backLeft2");
        DcMotor backRight3 = hardwareMap.dcMotor.get("backRight3");
        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft2.setDirection(DcMotor.Direction.REVERSE);
        DcMotor motorarm = hardwareMap.dcMotor.get("arm");
        motorarm.setDirection(DcMotor.Direction.REVERSE);
        motorarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotor extendarm = hardwareMap.dcMotor.get("extend");
        extendarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        intake = hardwareMap.get(Servo.class, "intake");
        DcMotor intakeCoreHex = hardwareMap.get(DcMotor.class, "intake");
        intakeCoreHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // TODO Adjust the orientation parameters to match our robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("frontLeft0:",frontLeft0.getCurrentPosition());
            telemetry.addData("frontRight1:",frontRight1.getCurrentPosition());
            telemetry.addData("backLeft2:",backLeft2.getCurrentPosition());
            telemetry.addData("backRight3:",backRight3.getCurrentPosition());
            telemetry.addData("arm:", motorarm.getCurrentPosition());
            telemetry.addData("extendArm:",extendarm.getCurrentPosition());
            telemetry.addData("intake:",intakeCoreHex.getCurrentPosition());
            telemetry.update();

        }

    }
}
