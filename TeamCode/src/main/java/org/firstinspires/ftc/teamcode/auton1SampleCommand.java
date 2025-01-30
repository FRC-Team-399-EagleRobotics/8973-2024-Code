package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

class Action{
    public Action() {
    }

    public boolean run(){
        return true;
    }
}
class Robot{
    IMU imu;
    int target;
    int ticksPerIn = 45;//188;
    DcMotor frontLeft0;
    DcMotor frontRight1;
    DcMotor backLeft2;
    DcMotor backRight3;
    DcMotor motorarm;
    DcMotor extendarm;
    Servo intake;
    DcMotor intakeCoreHex;
    Robot(HardwareMap hardwareMap) {
        frontLeft0 = hardwareMap.dcMotor.get("frontLeft0");
        frontRight1 = hardwareMap.dcMotor.get("frontRight1");
        backLeft2 = hardwareMap.dcMotor.get("backLeft2");
        backRight3 = hardwareMap.dcMotor.get("backRight3");
        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorarm = hardwareMap.dcMotor.get("arm");
        motorarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorarm.setDirection(DcMotorSimple.Direction.REVERSE);
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
        target = 0;
    }
}
class SequentialAction extends Action{
    private Action[] CurrentActions;
    private int index = 0;
    boolean done;
    public SequentialAction(HardwareMap hardwareMap, Action ... actions) {
        CurrentActions = actions;
    }
    @Override
    public boolean run(){
        done = CurrentActions[index].run();
        if (done){
            index++;
            if (index+1>CurrentActions.length){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }
}
class ArmUpdate extends Action{
    final double kPArm = 0.01;
    int armPos;
    int error;
    double armOutPower;
    Robot robot;

    ArmUpdate(Robot r) {
        robot = r;
    }

    @Override
    public boolean run(){
        armPos = robot.motorarm.getCurrentPosition();
        error = robot.target-armPos;
        armOutPower = error*kPArm;
        robot.motorarm.setPower(armOutPower);
        return false;
    }
}
class ArmScore extends Action{
    Robot robot;
    double accuracy;
    ArmScore(Robot r) {
        robot = r;
        accuracy = 20;
    }

    @Override
    public boolean run(){
        robot.target = 1406;
        if (robot.motorarm.getCurrentPosition()+accuracy> robot.target) {
            return true;
        }
        else{
            return false;

            }
    }
}
class Outake extends Action{
    Robot robot;
    Outake(Robot r){
        robot = r;
    }
    @Override
    public boolean run(){
        robot.intakeCoreHex.setPower(1);
        return true;
    }
}
class Drive extends Action{
    Robot robot;
    double position;
    double averagePos;
    boolean initialized;
    double turn;
    double power;
    Drive(Robot r, double pow, double pos,double t){
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

class Scheduler {
    ArrayList<Action> Actions = new ArrayList<>();
    Scheduler() {
    }
    void run(){
        ArrayList<Integer> toRemove = new ArrayList<>();
        for (int i=0; i<Actions.size(); i++){
            if (Actions.get(i).run()){
                toRemove.add(i);
            }
        }
        Collections.reverse(toRemove);
        for (int action : toRemove){
            Actions.remove(action);
        }

    }
    void add(Action action){
        Actions.add(action);
    }
}

@Autonomous(name = "autonC", group = "Autonomous")
public class auton1SampleCommand extends LinearOpMode {

    private PIDController armController;

    public double encoderToDegrees(double enc) {
        return (enc / 5.882) + 50-90;
    }



    @Override
    public void runOpMode() throws InterruptedException {
        armController = new PIDController(0.0005, 0, 0.0000001);



        waitForStart();
        if (isStopRequested()) return;
        boolean completed = false;
        Robot robot = new Robot(hardwareMap);
        Scheduler sceduler = new Scheduler();
        sceduler.add(new SequentialAction(hardwareMap,new Drive(robot,-0.5,4,0),new Drive(robot,-0.5,27,-0.6),new Drive(robot,-0.5,14,0), new ArmScore(robot), new Outake(robot)));
        sceduler.add(new ArmUpdate(robot));
//        sceduler.add(new Outake(robot));
        while (opModeIsActive()){
            sceduler.run();
            telemetry.addData("ArmPosition",robot.motorarm.getCurrentPosition());
            telemetry.update();
        }
    }
}