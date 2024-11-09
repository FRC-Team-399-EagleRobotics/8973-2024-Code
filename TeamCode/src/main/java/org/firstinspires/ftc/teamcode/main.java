package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class main extends LinearOpMode {
    DcMotor frontLeft0;
    DcMotor frontRight1;
    DcMotor backLeft2;
    DcMotor backRight3;
    DcMotor motorarm;
    Servo intake;

    public void drive(double power, double turn){
        double leftPower;
        double rightPower;
        leftPower = Range.clip(power+turn, -1, 1);
        rightPower = Range.clip(power-turn, -1, 1);
        frontLeft0.setPower(leftPower);
        frontRight1.setPower(rightPower);
        backLeft2.setPower(leftPower);
        backRight3.setPower(rightPower);



    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft0 = hardwareMap.dcMotor.get("frontLeft0");
        frontRight1 = hardwareMap.dcMotor.get("frontRight1");
        backLeft2 = hardwareMap.dcMotor.get("backLeft2");
        backRight3 = hardwareMap.dcMotor.get("backRight3");
        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorarm = hardwareMap.dcMotor.get("arm");
        intake = hardwareMap.get(Servo.class, "intake");


        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            float thr = gamepad1.left_trigger + gamepad1.right_trigger;
            float str = gamepad1.right_stick_x;
            drive(-thr, str);
            if (gamepad1.y){
                intake.setPosition(0);
                telemetry.addData("Buttonycoderunning","True");
            }
            else if (gamepad1.a) {
                intake.setPosition(1);
                telemetry.addData("Button A pressed","True");
            }
            else {
                intake.setPosition(0.545);
                telemetry.addData("nobuttonspressed","true");
            }
            telemetry.update();
        }

    }

}
