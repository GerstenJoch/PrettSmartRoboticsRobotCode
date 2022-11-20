package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class StandardIntake {
    private LinearOpMode myOpMode;
    public Servo gripper;
    public Servo swivel;
    int Gcount = 0;
    int Rcount = 0;
    boolean GdirectionState;
    boolean RdirectionState;
    boolean Gclickable = true;
    boolean Rclickable = true;
    public StandardIntake(LinearOpMode opmode) {myOpMode = opmode;}

    public void init() {
        gripper = myOpMode.hardwareMap.get(Servo.class, "intake");
        gripper.setPosition(.6);
        swivel = myOpMode.hardwareMap.get(Servo.class, "rotateIntake");
        swivel.setPosition(1);
        GdirectionState = false;
        RdirectionState = false;
    }

    public void gripper() {
        while (myOpMode.gamepad2.a) {
           if (Gclickable) {
               if (GdirectionState == false) {
                   if (myOpMode.gamepad2.a) {
                       Gcount++;
                       GdirectionState = true;
                       gripper.setPosition(.1);
                   }
               } else {
                   if (myOpMode.gamepad2.a) {
                       Gcount++;
                       GdirectionState = false;
                       gripper.setPosition(.6);
                   }
               }
               Gclickable = false;
           }
        }
        Gclickable = true;
    }
    public void rotate() {
        while (myOpMode.gamepad2.b) {
            if (Rclickable) {
                if (RdirectionState == false) {
                    if (myOpMode.gamepad2.b) {
                        Rcount++;
                        RdirectionState = true;
                        swivel.setPosition(0);
                    }
                } else {
                    if (myOpMode.gamepad2.b) {
                        Rcount++;
                        RdirectionState = false;
                        swivel.setPosition(1);
                    }
                }
                Rclickable = false;
            }
        }
        Rclickable = true;
        if (myOpMode.gamepad2.y) {
            swivel.setPosition(.5);
        }
    }
}
