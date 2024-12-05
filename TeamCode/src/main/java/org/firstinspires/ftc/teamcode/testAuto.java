package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
@Autonomous(name="testauto")
public class testAuto extends LinearOpMode {
    DcMotor frontLeft0;
    DcMotor frontRight1;
    DcMotor backLeft2;
    DcMotor backRight3;
    DcMotor motorArm;
    DcMotor extendArm;
    DcMotor intakeCoreHex;
    public final int ARM_TOP_POS = 1000;

    public void setArmPosition(int position){
        motorArm.setTargetPosition(position);

        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorArm.setPower(0.5);
    }
    public void setExtrustionPower(double power){
        extendArm.setPower(power);
        sleep(1000);
        extendArm.setPower(0);
    }
    public void intake(){
        intakeCoreHex.setPower(1);
        sleep(1000);

    }
    public void outake(){
        intakeCoreHex.setPower(-1);
        sleep(3000);
    }
    @Override
    public void runOpMode() throws InterruptedException {
//        frontLeft0 = hardwareMap.dcMotor.get("frontLeft0");
//        frontRight1 = hardwareMap.dcMotor.get("frontRight1");
//        backLeft2 = hardwareMap.dcMotor.get("backLeft2");
//        backRight3 = hardwareMap.dcMotor.get("backRight3");
//        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
//        backLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorArm = hardwareMap.dcMotor.get("arm");
        motorArm.setDirection(DcMotor.Direction.REVERSE);
//
        extendArm = hardwareMap.dcMotor.get("extend");
        intakeCoreHex = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            sleep(1000);
            setArmPosition(ARM_TOP_POS);


            sleep(1000);
            setArmPosition(60);
//            setArmPosition(20);
            sleep(1000);
            setArmPosition(ARM_TOP_POS);
            sleep(1000);
            setExtrustionPower(0.8);
            sleep(1000);
            setExtrustionPower(-1000);
            sleep(1000);
            intake();
            sleep(1000);
            outake();
            sleep(3000);




        }

    }}
