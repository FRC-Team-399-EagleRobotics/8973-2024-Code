package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

enum Positions{
    FLOOR,
    STOW,
    BUCKET

}
@TeleOp
public class main extends LinearOpMode {
    DcMotor frontLeft0;
    DcMotor frontRight1;
    DcMotor backLeft2;
    DcMotor backRight3;
    DcMotor motorarm;
    DcMotor extendarm;
    Servo intake;
    DcMotor intakeCoreHex;

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
    public void setArmPosition(Positions position){
        switch(position){
            case FLOOR:

                break;
            case STOW:

                break;
            case BUCKET:

                break;
            default:
                break;
        }
        motorarm.setTargetPosition(0);

    }
    public void grab



    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft0 = hardwareMap.dcMotor.get("frontLeft0");
        frontRight1 = hardwareMap.dcMotor.get("frontRight1");
        backLeft2 = hardwareMap.dcMotor.get("backLeft2");
        backRight3 = hardwareMap.dcMotor.get("backRight3");
        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorarm = hardwareMap.dcMotor.get("arm");
        extendarm = hardwareMap.dcMotor.get("extend");
//        intake = hardwareMap.get(Servo.class, "intake");
        intakeCoreHex = hardwareMap.get(DcMotor.class, "intake");
        intakeCoreHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            float thr = gamepad1.right_trigger - gamepad1.left_trigger;
            float str = gamepad1.right_stick_x;
            drive(thr, str);
//            if (gamepad1.y){
//                intake.setPosition(0);
//                telemetry.addData("Buttonycoderunning","True");
//            }
//            else if (gamepad1.a) {
//                intake.setPosition(1);
//                telemetry.addData("Button A pressed","True");
//            }
//            else {
//                intake.setPosition(0.545);
//                telemetry.addData("nobuttonspressed","true");
//            }
            if (gamepad2.dpad_down) {
                while(intakeCoreHex.getCurrentPosition()<1000) {
                    telemetry.addData("Position", intakeCoreHex.getCurrentPosition());
                    intakeCoreHex.setPower(1);
                }
            }
            else if (gamepad2.dpad_up) {
                intakeCoreHex.setPower(-1);
            }
            if(gamepad1.a){

            }
            else {
                intakeCoreHex.setPower(0);
            }

            motorarm.setPower(gamepad2.left_stick_y);
            extendarm.setPower(-gamepad2.right_stick_y);
            telemetry.update();
        }

    }

}
