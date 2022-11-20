package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class IntakeArm {
    private LinearOpMode myOpMode;
    private DcMotor lift_puller;

    public IntakeArm(LinearOpMode opmode) {myOpMode = opmode;}

    public void init() {
        lift_puller = myOpMode.hardwareMap.get(DcMotor.class, "lift");
    }

    public void lift() {
        if (myOpMode.gamepad2.right_stick_y != 0) {
            motor_power(lift_puller, myOpMode.gamepad2.right_stick_y * 1.5);
        }else {
            motor_power(lift_puller, 0);
        }

    }

    public void motor_power(DcMotor motor, double speed) {
        motor.setPower(speed);
    }
}
