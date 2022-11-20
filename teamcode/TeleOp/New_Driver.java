package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Barrier;
import org.firstinspires.ftc.teamcode.StandardIntake;
import org.firstinspires.ftc.teamcode.IntakeArm;
import org.firstinspires.ftc.teamcode.MechanumDrive;

@TeleOp

public class New_Driver extends LinearOpMode {

    MechanumDrive drive = new MechanumDrive(this);
      IntakeArm arm = new IntakeArm(this);
      StandardIntake intake = new StandardIntake(this);
      Barrier barrier = new Barrier(this);
    @Override
    public void runOpMode() throws InterruptedException {
        drive.init();
        arm.init();
        intake.init();
        barrier.init();
        waitForStart();

        while (opModeIsActive()) {
            drive.FieldCentric(.75);
            intake.gripper();
            arm.lift();
            intake.rotate();
            barrier.rotate();

        }

    }
}
