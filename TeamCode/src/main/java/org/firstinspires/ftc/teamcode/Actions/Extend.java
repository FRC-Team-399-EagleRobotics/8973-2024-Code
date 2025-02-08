package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Extend extends Action {
    Robot robot;
    double targetTime;
    boolean initialized;
    ElapsedTime timer;
    public Extend(Robot r, double target){
        robot = r;
        targetTime = target;
        initialized = false;
    }
    @Override
    public boolean run(){
        if(!initialized){
            timer = new ElapsedTime();
            robot.extendarm.setPower(1);
            initialized = true;
        }
      if (timer.seconds()>targetTime){
        robot.extendarm.setPower(0);
        return true;
      }
      else{
        return false;
      }   
    }
}
