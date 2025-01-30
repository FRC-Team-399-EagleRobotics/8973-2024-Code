package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.Actions.Action;
import org.firstinspires.ftc.teamcode.Actions.Robot;

public class Outake extends Action {
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
