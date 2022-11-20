package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Barrier {
    private LinearOpMode myOpMode;
    public Servo barrier;
    int count = 0;
    boolean directionState;
    boolean clickable = true;

    public Barrier(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        barrier = myOpMode.hardwareMap.get(Servo.class, "barriers");
        barrier.setPosition(1);
        directionState = false;
    }

    public void rotate() {
        while (myOpMode.gamepad2.x) {
            if (clickable) {
                if (directionState == false) {
                    if (myOpMode.gamepad2.x) {
                        count++;
                        directionState = true;
                        barrier.setPosition(0);
                    }
                } else {
                    if (myOpMode.gamepad2.x) {
                        count++;
                        directionState = false;
                        barrier.setPosition(1);
                    }
                }
                clickable = false;

            }
        }
        clickable = true;
    }
}
