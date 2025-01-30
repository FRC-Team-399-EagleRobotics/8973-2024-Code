package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.Actions.Action;
import org.firstinspires.ftc.teamcode.Actions.Robot;

public class ArmScore extends Action {
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
