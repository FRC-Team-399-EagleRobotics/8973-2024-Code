package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "potentialAuton", group = "Autonomous")
public class potentialAuton extends LinearOpMode {
    IMU imu;
    int target = 0;
    DcMotor frontLeft0;
    DcMotor frontRight1;
    DcMotor backLeft2;
    DcMotor backRight3;
    DcMotor motorarm;
    DcMotor extendarm;
    Servo intake;
    DcMotor intakeCoreHex;
    private PIDController armController;

    public void setArmPosition(int target){
        int accuracy = 20;
        int armPos = motorarm.getCurrentPosition();
        while (Math.abs(target - armPos) > accuracy) {

            double pid = armController.calculate(armPos, target);

            double ff = Math.cos(Math.toRadians(encoderToDegrees(armPos))) * 0.025;
            motorarm.setPower(ff + pid);
        }
    }
    public double encoderToDegrees(double enc) {
        return (enc / 5.882) + 50-90;
    }


    public void drive(double power, double turn) {
        double leftPower;
        double rightPower;
        leftPower = Range.clip(power + turn, -1, 1);
        rightPower = Range.clip(power - turn, -1, 1);
        frontLeft0.setPower(leftPower);
        frontRight1.setPower(rightPower);
        backLeft2.setPower(leftPower);
        backRight3.setPower(rightPower);


    }
    public void driveToPosition(double power, int position){
        frontLeft0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while ((frontLeft0.getCurrentPosition() + frontRight1.getCurrentPosition() + backLeft2.getCurrentPosition() + backRight3.getCurrentPosition()) / 4 < position) {
            drive(power, 0);
        }
        drive(0, 0);
    }
    public double getAngle(){

        double heading= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Heading", heading);
        telemetry.update();
        return heading;
    }

    public void driveToAngle(double power, int degrees){
        frontLeft0.setPower(power);
        backLeft2.setPower(power);
        frontRight1.setPower(-power);
        backRight3.setPower(-power);
        while(getAngle()>degrees){}
        drive(0, 0);


    }


    @Override
    public void runOpMode() throws InterruptedException {
        armController = new PIDController(0.0005, 0, 0.0000001);


        frontLeft0 = hardwareMap.dcMotor.get("frontLeft0");
        frontRight1 = hardwareMap.dcMotor.get("frontRight1");
        backLeft2 = hardwareMap.dcMotor.get("backLeft2");
        backRight3 = hardwareMap.dcMotor.get("backRight3");
        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorarm = hardwareMap.dcMotor.get("arm");
        motorarm.setDirection(DcMotor.Direction.REVERSE);
        motorarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendarm = hardwareMap.dcMotor.get("extend");
//        intake = hardwareMap.get(Servo.class, "intake");
        intakeCoreHex = hardwareMap.get(DcMotor.class, "intake");
        intakeCoreHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu = hardwareMap.get(IMU.class, "imu");
        // TODO Adjust the orientation parameters to match our robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            driveToPosition(0.4, 40);
            driveToAngle(0.3, 90);
            driveToPosition(0.4, 1000);
            driveToAngle(0.3, 135);
            setArmPosition(990);
            extendarm.setPower(0.5);
            sleep(1500);
            extendarm.setPower(0);
//            driveToPosition(0.4, 50);


            intakeCoreHex.setPower(-1);
            sleep(1500);
            intakeCoreHex.setPower(0);




        }


    }
}