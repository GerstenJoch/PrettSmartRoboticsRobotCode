/*
Copyright 2021 FIRST Tech Challenge Team 16441

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class Driver extends OpMode {
    /* Declare OpMode members. */
    private Blinker control_Hub__2_;
    private Blinker expansion_Hub__7_;
    private HardwareDevice webcam;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor gripperArm;
    private Servo gripperServo;
    private Gyroscope imu;
    private DcMotor rearleft;
    private DcMotor rearright;
    private ColorSensor sensorLeft;
    private ColorSensor sensorRight;
    private DcMotor shooter;
    private Servo shooterServo;
    private DcMotor wobbleGripper;
    private Servo wobbleServo;
    private AnalogInput shooterPowerAdjust;

    double powerFL;
    double powerFR;
    double powerRR;
    double powerRL;
    double speed;
    long startTime;
    final double OPENED = 1;
    final double CLOSED = 0;
    boolean isShooterOn;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        control_Hub__2_ = hardwareMap.get(Blinker.class, "Control Hub (2)");
        expansion_Hub__7_ = hardwareMap.get(Blinker.class, "Expansion Hub (7)");
        webcam = hardwareMap.get(HardwareDevice.class, "Webcam");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        rearleft = hardwareMap.get(DcMotor.class, "rearleft");
        rearright = hardwareMap.get(DcMotor.class, "rearright");
        sensorLeft = hardwareMap.get(ColorSensor.class, "sensorLeft");
        sensorRight = hardwareMap.get(ColorSensor.class, "sensorRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterPowerAdjust = hardwareMap.get(AnalogInput.class, "shooterPowerAdjust");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        wobbleGripper = hardwareMap.get(DcMotor.class, "wobbleGripper");
        gripperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGripper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        isShooterOn = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        powerRL = (gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x;
        powerFL = (gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x;
        powerRR = ((-gamepad1.left_stick_y) + gamepad1.left_stick_x) + gamepad1.right_stick_x;
        powerFR = ((-gamepad1.left_stick_y) - gamepad1.left_stick_x) + gamepad1.right_stick_x;

        speed = -0.8;
        frontleft.setPower(powerFL*speed);
        frontright.setPower(powerFR*speed);
        rearleft.setPower(powerRL*speed);
        rearright.setPower(powerRR*speed);

        if(gamepad2.right_bumper){
            isShooterOn=true;
        }

        if(gamepad2.left_bumper){
            isShooterOn=false;
        }

        if (isShooterOn){
            shooter.setPower(-shooterPowerAdjust.getVoltage()/shooterPowerAdjust.getMaxVoltage());
        } else {
            shooter.setPower(0);
        }

        if(gamepad2.right_trigger>0){
            shooterServo.setPosition(0);
            pause(100);
            shooterServo.setPosition(1);
            pause(500);
            shooterServo.setPosition(-0.1);
            pause(300);
        }

        if(gamepad2.a){
            //gripperServo.setPosition(OPENED);
            gripperArm.setPower(1);
        }

        if(gamepad2.b){
            //gripperServo.setPosition(CLOSED);
            gripperArm.setPower(0);
        }

        if(gamepad2.y){
            wobbleServo.setPosition(OPENED);
        }

        if(gamepad2.x){
            wobbleServo.setPosition(CLOSED);
        }

        //gripperArm.setPower(-gamepad2.right_stick_y*0.5);

        wobbFileGripper.setPower(gamepad2.left_stick_y*0.8);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    void pause(int pauseMilliSeconds){
        startTime =System.currentTimeMillis();
        while (System.currentTimeMillis()-startTime < pauseMilliSeconds){

        }
    }
}
