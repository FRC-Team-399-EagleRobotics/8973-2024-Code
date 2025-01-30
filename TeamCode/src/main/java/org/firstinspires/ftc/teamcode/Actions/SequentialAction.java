package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SequentialAction extends Action{
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
