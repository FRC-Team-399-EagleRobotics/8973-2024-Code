package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.Actions.Action;
import org.firstinspires.ftc.teamcode.Actions.Robot;

public class ArmUpdate extends Action {
    final double kPArm = 0.01;
    int armPos;
    int error;
    double armOutPower;
    Robot robot;

    public ArmUpdate(Robot r) {
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
