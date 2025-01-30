//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.drive.Drive;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Actions.Robot;
//import org.firstinspires.ftc.teamcode.Actions.ArmScore;
//import org.firstinspires.ftc.teamcode.Actions.ArmUpdate;
//import org.firstinspires.ftc.teamcode.Actions.Outake;
//import org.firstinspires.ftc.teamcode.Actions.Sceduler;
//import org.firstinspires.ftc.teamcode.Actions.SequentialAction;
//
//
//@Autonomous(name = "autonC", group = "Autonomous")
//public class auton1SampleCommandTimer extends LinearOpMode {
//
//    private PIDController armController;
//
//    public double encoderToDegrees(double enc) {
//        return (enc / 5.882) + 50-90;
//    }
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        armController = new PIDController(0.0005, 0, 0.0000001);
//
//
//
//        waitForStart();
//        if (isStopRequested()) return;
//        boolean completed = false;
//        Robot robot = new Robot(hardwareMap);
//        Sceduler sceduler = new Sceduler();
//        sceduler.add(new SequentialAction(hardwareMap,new Drive(robot,-0.5,4,0),new Drive(robot,-0.5,27,-0.6),new Drive(robot,-0.5,14,0), new ArmScore(robot), new Outake(robot)));
//        sceduler.add(new ArmUpdate(robot));
////        sceduler.add(new Outake(robot));
//        while (opModeIsActive()){
//            sceduler.run();
//            telemetry.addData("ArmPosition",robot.motorarm.getCurrentPosition());
//            telemetry.update();
//        }
//    }
//}