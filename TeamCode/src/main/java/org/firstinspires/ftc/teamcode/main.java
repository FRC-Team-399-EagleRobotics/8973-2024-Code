package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class main extends LinearOpMode {
    DcMotor frontLeft0;
    DcMotor frontRight1;
    DcMotor backLeft2;
    DcMotor backRight3;
    DcMotor motorarm;

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
        frontLeft0 = hardwareMap.dcMotor.get("drmotor0");
        frontRight1 = hardwareMap.dcMotor.get("drmotor1");
        backLeft2 = hardwareMap.dcMotor.get("drmotor2");
        backRight3 = hardwareMap.dcMotor.get("drmotor3");
        motorarm = hardwareMap.dcMotor.get("arm");


        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            drive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

    }

}
