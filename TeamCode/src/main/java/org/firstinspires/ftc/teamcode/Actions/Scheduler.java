package org.firstinspires.ftc.teamcode.Actions;

import java.util.ArrayList;
import java.util.Collections;

public class Sceduler {
    ArrayList<Action> Actions = new ArrayList<>();
    public Sceduler() {
    }
    public void run(){
        ArrayList<Integer> toRemove = new ArrayList<>();
        for (int i=0; i<Actions.size(); i++){
            if (Actions.get(i).run()){
                toRemove.add(i);
            }
        }
        Collections.reverse(toRemove);
        for (int action : toRemove){
            Actions.remove(action);
        }

    }
    public void add(Action action){
        Actions.add(action);
    }
}
