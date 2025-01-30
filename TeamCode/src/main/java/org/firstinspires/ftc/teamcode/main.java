package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//if else
//enum Positions{
//    FLOOR,
//    STOW,
//    BUCKET
//
//}
@TeleOp
public class main extends LinearOpMode {
    int target = 0;
    int error;
    double kPArm = 0.01;//TODO decreace this to fix arm jitter
    DcMotor frontLeft0;
    DcMotor frontRight1;
    DcMotor backLeft2;
    DcMotor backRight3;
    DcMotor motorarm;
    DcMotor extendarm;
    Servo intake;
    DcMotor intakeCoreHex;
    //private PIDController armController;

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
//    public void setArmPosition(Positions position){
//        switch(position){
//            case FLOOR:
//
//                break;
//            case STOW:
//
//                break;
//            case BUCKET:
//
//                break;
//            default:
//                break;
//        }
//        motorarm.setTargetPosition(0);
//
//    }



    @Override
    public void runOpMode() throws InterruptedException {
        //armController = new PIDController(0.0005, 0, 0.0000001);


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

        extendarm = hardwareMap.dcMotor.get("extend");
//        intake = hardwareMap.get(Servo.class, "intake");
        intakeCoreHex = hardwareMap.get(DcMotor.class, "intake");
        intakeCoreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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

                    intakeCoreHex.setPower(-1);

            }
            else if (gamepad2.dpad_up) {
                intakeCoreHex.setPower(1);
            }



            else {

                intakeCoreHex.setPower(0);
            }
            telemetry.addData("Arm", motorarm.getCurrentPosition());
            telemetry.addData("drive",frontLeft0.getCurrentPosition());
            if(gamepad2.a) {
                target = 0;//STOW
            } else if(gamepad2.b) {
                target = 1406;//1125/20*25;//TODO change this to arm score location
            } else if(gamepad2.x) {
                target = 200;//160;//TODO change this to correct ground
            }else if(gamepad2.y) {
                target = -625;//500;
            }


            int armPos = motorarm.getCurrentPosition();
            error = target-armPos;
            double armOutPower = error*kPArm;

            //double pid = armController.calculate(armPos, target);

            //double ff = Math.cos(Math.toRadians(encoderToDegrees(target)))*0.025;
            //motorarm.setPower(pid);
            telemetry.addData("Encodertodegrees", (encoderToDegrees(armPos)));
            //telemetry.addData("ff", ff);
            motorarm.setPower(armOutPower);
            //motorarm.setPower(gamepad2.left_stick_y);
            extendarm.setPower(-gamepad2.right_stick_y);
            telemetry.update();
        }

    }

    public double encoderToDegrees(double enc) {
        return (enc / 5.882) + 50-90;
    }

}
