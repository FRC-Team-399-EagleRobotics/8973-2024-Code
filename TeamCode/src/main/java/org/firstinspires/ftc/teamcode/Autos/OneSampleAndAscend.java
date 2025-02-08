package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.ArmLevel1Ascend;
import org.firstinspires.ftc.teamcode.Actions.ArmScore;
import org.firstinspires.ftc.teamcode.Actions.ArmUpdate;
import org.firstinspires.ftc.teamcode.Actions.Drive;
import org.firstinspires.ftc.teamcode.Actions.Extend;
import org.firstinspires.ftc.teamcode.Actions.Robot;
import org.firstinspires.ftc.teamcode.Actions.Scheduler;
import org.firstinspires.ftc.teamcode.Actions.SequentialAction;
import org.firstinspires.ftc.teamcode.Actions.TimedOutake;

@Autonomous(name = "1SampleParkAscend", group = "Autonomous")
public class OneSampleAndAscend extends LinearOpMode {

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
                new Drive(robot,-0.5,20,-0.6),
                new Drive(robot,-0.5,14,0),
                new ArmScore(robot),
                new TimedOutake(robot,2),
                new Drive(robot, 0.5, 30,-0.1),
                new Drive(robot, 0.5,36,0.2),
                new ArmLevel1Ascend(robot),
                new Extend(robot, 2)
        )
        );
        sceduler.add(new ArmUpdate(robot));
//        sceduler.add(new Outake(robot));
        while (opModeIsActive()){
            sceduler.run();
            telemetry.addData("ArmPosition",robot.motorarm.getCurrentPosition());
            telemetry.update();
        }
    }
                }
