package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.IntakeArm;
import org.firstinspires.ftc.teamcode.MechanumDrive;
import org.firstinspires.ftc.teamcode.StandardIntake;

@Autonomous

public class New_Autonomous extends LinearOpMode {

    MechanumDrive drive = new MechanumDrive(this);
    IntakeArm arm = new IntakeArm(this);
    StandardIntake intake = new StandardIntake(this);

    @Override
    public void runOpMode() {
        drive.init();
        arm.init();
        intake.init();

        waitForStart();

        while (opModeIsActive()) {
            drive.reset_odometry();
            double y = drive.get_odometry_y_In();
            while (y < 1000) {
                System.out.println(drive.get_odometry_y_In());
                drive.Forward(0.4);
                y = drive.get_odometry_y_In();
            }
            drive.Stop();


            /*
            if Tile A5 or F2:
                 if location 1:
                    Forward,
                    Right,
                    Stop,
                    Extend lift,
                    Forward,
                    Stop,
                    Set intake to cone drop,
                    Backward,
                    Left,
                    Stop,
                    Return lift to normal.
                if location 2:
                    Forward,
                    Right,
                    Stop,
                    Extend lift,
                    Forward,
                    Stop,
                    Set intake to cone drop,
                    Backward,
                    Left,
                    Stop,
                    Return lift to normal.
                 if location 3:
                    Forward,
                    Right,
                    Stop,
                    Extend lift,
                    Forward,
                    Stop,
                    Set intake to cone drop,
                    Backward,
                    Right,
                    Stop,
                    Return lift to normal.

           if Tile A2 or F5:
                 if location 1:
                    Forward,
                    Left,
                    Stop,
                    Extend lift,
                    Forward,
                    Stop,
                    Set intake to cone drop,
                    Backward,
                    Left,
                    Stop,
                    Return lift to normal.
                if location 2:
                    Forward,
                    Left,
                    Stop,
                    Extend lift,
                    Forward,
                    Stop,
                    Set intake to cone drop,
                    Backward,
                    Left,
                    Stop,
                    Return lift to normal,
                 if location 3:
                    Forward,
                    Left,
                    Stop,
                    Extend lift,
                    Forward,
                    Stop,
                    Set intake to cone drop,
                    Backward,
                    Right,
                    Stop,
                    Return lift to normal.



            if Simple that works for all:
                 if location 1: Forward, left,stop in ratio 5:3
                 if location 2: Forward, stop
                 if location 3: forward, right, stop in ratio 5:3

             */

        }


    }
}
