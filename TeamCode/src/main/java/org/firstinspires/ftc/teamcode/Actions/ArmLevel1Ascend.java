package org.firstinspires.ftc.teamcode.Actions;

public class ArmLevel1Ascend extends Action {
    Robot robot;
    double accuracy;
    public ArmLevel1Ascend(Robot r) {
        robot = r;
        accuracy = 20;
    }

    @Override
    public boolean run(){
        robot.target = 580;
        if (robot.motorarm.getCurrentPosition()+accuracy> robot.target) {
            return true;
        }
        else{
            return false;

        }
    }
}
