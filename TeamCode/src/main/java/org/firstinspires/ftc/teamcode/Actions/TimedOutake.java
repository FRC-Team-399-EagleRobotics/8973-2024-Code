package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.Action;
import org.firstinspires.ftc.teamcode.Actions.Robot;

public class TimedOutake extends Action {
    Robot robot;
    double targetTime;
    boolean initialized;
    ElapsedTime timer;
    public TimedOutake(Robot r, double target){
        robot = r;
        targetTime = target;
        initialized = false;
    }
    @Override
    public boolean run(){
        if(!initialized){
            timer = new ElapsedTime();
            robot.intakeCoreHex.setPower(1);
            initialized = true;
        }
      if (timer.seconds()>targetTime){
        robot.intakeCoreHex.setPower(0);
        return true;
      }
      else{
        return false;
      }   
    }
}
