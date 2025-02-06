package org.firstinspires.ftc.teamcode.Autos;

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

import org.firstinspires.ftc.teamcode.Actions.ArmScore;
import org.firstinspires.ftc.teamcode.Actions.ArmUpdate;
import org.firstinspires.ftc.teamcode.Actions.Outake;
import org.firstinspires.ftc.teamcode.Actions.Scheduler;
import org.firstinspires.ftc.teamcode.Actions.SequentialAction;
import org.firstinspires.ftc.teamcode.Actions.Robot;
import org.firstinspires.ftc.teamcode.Actions.Drive;
import org.firstinspires.ftc.teamcode.Actions.TimedOutake;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Autonomous(name = "1SamplePark", group = "Autonomous")
public class OneSampleAndPark extends LinearOpMode {

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
        sceduler.add(new SequentialAction(hardwareMap,
                new Drive(robot,-0.5,4,0),
                new Drive(robot,-0.5,27,-0.6),
                new Drive(robot,-0.5,14,0),
                new ArmScore(robot),
                new TimedOutake(robot,2)));
        sceduler.add(new ArmUpdate(robot));
//        sceduler.add(new Outake(robot));
        while (opModeIsActive()){
            sceduler.run();
            telemetry.addData("ArmPosition",robot.motorarm.getCurrentPosition());
            telemetry.update();
        }
    }
                }
