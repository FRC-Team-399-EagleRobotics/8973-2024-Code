package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Actions.Action;
import org.firstinspires.ftc.teamcode.Actions.Robot;

public class Drive extends Action {
    Robot robot;
    double position;
    double averagePos;
    boolean initialized;
    double turn;
    double power;
    public Drive(Robot r, double pow, double pos, double t){
        robot  = r;
        initialized = false;
        power = pow;
        position = pos;
        turn = t;

    }
    @Override
    public boolean run(){
        if (!initialized){
            double leftPower;
            double rightPower;
//            leftPower = Range.clip(power + turn, -1, 1);
//            rightPower = Range.clip(power - turn, -1, 1);
            leftPower = power + turn;
            rightPower = power - turn;

            robot.frontLeft0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            averagePos = 0;
            robot.frontLeft0.setPower(leftPower);
            robot.frontRight1.setPower(rightPower);
            robot.backLeft2.setPower(leftPower);
            robot.backRight3.setPower(rightPower);

            initialized = true;
        }
        averagePos = (Math.abs(robot.frontLeft0.getCurrentPosition()) + Math.abs(robot.frontRight1.getCurrentPosition()))/2.0;
//            telemetry.addData("averagePos", averagePos);
//            telemetry.addData("target",position*ticksPerIn);
//            telemetry.update();
        if (averagePos > position*robot.ticksPerIn){
            robot.frontLeft0.setPower(0);
            robot.frontRight1.setPower(0);
            robot.backLeft2.setPower(0);
            robot.backRight3.setPower(0);
            return true;
        }
        else{
            return false;
        }
    }

}
