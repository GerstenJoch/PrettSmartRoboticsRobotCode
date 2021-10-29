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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.io.Serializable;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
import java.util.List;
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

public class FieldCentricTest extends OpMode {
    /* Declare OpMode members. */
    private Blinker control_Hub__2_;
    private Blinker expansion_Hub__7_;
    private DcMotor frontleft;
    private DcMotor frontright;
    //private Gyroscope imu_1;
    //private Gyroscope imu;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor intake;
    private DcMotor intakeArm;
    private DcMotor caroussel;
    private Servo servo;
    double powerFL;
    double powerFR;
    double powerBR;
    double powerBL;
    double speed;
    double FWD;
    double STR;
    double ROT;
    double carousselPower;
    double intakePower;
    double intakeArmPower;
    double theta;
    int position;
    String intakeTelemetry;
    String intakeArmTelemetry;
    String carousselTelemetry;

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        control_Hub__2_ = hardwareMap.get(Blinker.class, "Control Hub (2)");
        expansion_Hub__7_ = hardwareMap.get(Blinker.class, "Expansion Hub (7)");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");

        //imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        caroussel = hardwareMap.get(DcMotor.class, "caroussel");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");
        servo = hardwareMap.get(Servo.class, "servo1");
        backleft = hardwareMap.get(DcMotor.class, "rearleft");
        backright = hardwareMap.get(DcMotor.class, "rearright");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("IMU calibration ", "IN PROGRESS...");
        telemetry.update();



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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //powerBL = (gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerFL = (gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerBR = ((-gamepad1.left_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerFR = ((-gamepad1.left_stick_y) - gamepad1.left_stick_x) - gamepad1.right_stick_x;

        //Driving Mechanism
        theta = getCurrentHeading()+(3.1415/2);
        FWD = (gamepad1.left_stick_x*Math.sin(theta) + gamepad1.left_stick_y*Math.cos(theta));
        STR = (gamepad1.left_stick_x*Math.cos(theta) - gamepad1.left_stick_y*Math.sin(theta));
        ROT = gamepad1.right_stick_x;

        speed = -1;
        frontleft.setPower((FWD+STR+ROT)*speed);
        frontright.setPower((FWD-STR+ROT)*speed);
        backleft.setPower((-1*FWD-STR+ROT)*speed);
        backright.setPower((-1*FWD+STR+ROT)*speed);
        //Caroussel Mechanism
        if (gamepad2.a){
            carousselPower = 0.3;
            caroussel.setPower(carousselPower);
        }else{
            carousselPower = 0;
            caroussel.setPower(carousselPower);
        }
        position = intakeArm.getCurrentPosition();
        //Distance Sensor for the arm

        //IntakeArm Mechanism
        if (gamepad2.dpad_up){
            intakeArm.setPower(-0.3);
            if (position >= 90 && position <= 100){
                intakeArm.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            }            servo.setPosition(90);
            intakeArm.setPower(0);
        }if (gamepad2.dpad_down){
            intakeArm.setPower(0.3);
            if (position >= 90 && position <= 100){
                intakeArm.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            }            servo.setPosition(90);
            intakeArm.setPower(0);
        }

        
            
        /*Intake Mechanism
        if (gamepad2.b && intakePower==0){
            intakePower = 0.1;
            intake.setPower(intakePower);
        }if (gamepad2.b && intakePower !=0){
            intakePower = 0;
            intake.setPower(intakePower);
        }*/

        //Adding data to screen
        if (carousselPower == 0){
            carousselTelemetry = "isNotRunning";
        }else {
            carousselTelemetry = "isRunning";
        } if (intakeArmPower == 0){
            intakeArmTelemetry = "isNotRunning";
        }else {
            intakeArmTelemetry = "isRunning";
        } /*if (intakePower == 0){
            intakeTelemetry = "isNotRunning";
        }else {
            intakeTelemetry = "isRunning";}*/

        telemetry.addData("FWD ", FWD);
        telemetry.addData("STR ", STR);
        telemetry.addData("ROT ", ROT);
        telemetry.addData("Caroussel ", carousselTelemetry);
        //telemetry.addData("Intake", intakeTelemetry);
        telemetry.addData("IntakeArmPosition", intakeArmTelemetry);

        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    private double getCurrentHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle);
    }
}
