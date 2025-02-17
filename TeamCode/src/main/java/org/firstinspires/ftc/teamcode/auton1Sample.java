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
@Autonomous(name = "auton1Sample", group = "Autonomous")
public class auton1Sample extends LinearOpMode {
    IMU imu;
    int target = 0;
    int ticksPerIn = 45;//188;
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
        double kPArm = 0.01;
        int armPos = motorarm.getCurrentPosition();
        int error = target-armPos;
        double armOutPower = error*kPArm;
        while (opModeIsActive()&&(Math.abs(error) > accuracy)) {
            armPos = motorarm.getCurrentPosition();
            error = target-armPos;
            armOutPower = error*kPArm;
            motorarm.setPower(armOutPower);
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
        double averagePos =  (Math.abs(frontLeft0.getCurrentPosition()) + Math.abs(frontRight1.getCurrentPosition()))/2.0;
        while (opModeIsActive() && (averagePos < position*ticksPerIn)) {
            drive(power, 0);
            averagePos = (Math.abs(frontLeft0.getCurrentPosition()) + Math.abs(frontRight1.getCurrentPosition()))/2.0;
            telemetry.addData("averagePos", averagePos);
            telemetry.addData("target",position*ticksPerIn);
            telemetry.update();
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
        int accuracy = 5;
        while(opModeIsActive()&&(Math.abs(getAngle()-degrees))>accuracy){}
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
        boolean completed = false;
        while (opModeIsActive() && !completed) {
            driveToPosition(-0.1,6);
            driveToAngle(0.5,90);
            driveToPosition(-0.5,24);
            driveToAngle(0.5,135);
            setArmPosition(1406);
            intakeCoreHex.setPower(1);
            sleep(1000);
            completed = true;
//            driveToPosition(0.4, 40);
//            driveToAngle(0.3, 90);
//            driveToPosition(0.4, 1000);
//            driveToAngle(0.3, 135);
//            setArmPosition(990);
//            extendarm.setPower(0.5);
//            sleep(1500);
//            extendarm.setPower(0);
////            driveToPosition(0.4, 50);
//
//
//            intakeCoreHex.setPower(-1);
//            sleep(1500);
//            intakeCoreHex.setPower(0);




        }


    }
}