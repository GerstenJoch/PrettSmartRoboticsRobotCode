package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonomousBlueWarehouse extends LinearOpMode {
    public DcMotor frontleft;
    public DcMotor frontright;
    public DcMotor backleft;
    public DcMotor backright;
    public DcMotor caroussel;
    public DcMotor intake;
    public DcMotor intakeArm;
    double theta;
    double FWD;
    double STR;
    double ROT;
    int x = 1;
    BNO055IMU imu;
    Orientation angles;
    @Override
    public void runOpMode(){
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "rearleft");
        backright = hardwareMap.get(DcMotor.class, "rearright");
        caroussel = hardwareMap.get(DcMotor.class, "caroussel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");
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
        waitForStart();
        while (opModeIsActive()){ //Negative value intakeArm & intake = out and Positive = in
            // first value for power motor, 2nd for time running motor, 3rd for how long motor stops and goes to next function
            if (x == 1) //value check to let it only run once and when last function is called, the statement is set to false by changing x value 
            {
            intakeArmSpin(0.6, 1400, 100);
            forward(0.4, 870, 100);
            intakeSpin(-1, 1000, 100);
            backward(0.4,200, 10);
            intakeArmSpin(-0.6, 1400, 50);
            backward(0.4, 650, 50);
            rotateLeft(0.4,900, 100);
            backward(0.4,500, 100);
            left(0.8,1500,20);
            forward(0.6,1000, 300);
            x = 0;
            }
        }
    }//Methods for running motors

    void engine(double speed, double x, double y, double r){
        theta = getCurrentHeading()+(3.1415/2);
        FWD = (x*Math.sin(theta) + y*Math.cos(theta));
        STR = (x*Math.cos(theta) - y*Math.sin(theta));
        ROT = r;

        frontleft.setPower((FWD+STR+ROT)*(-1*speed));
        frontright.setPower((FWD-STR+ROT)*(-1*speed));
        backleft.setPower((-1*FWD-STR+ROT)*(-1*speed));
        backright.setPower((-1*FWD+STR+ROT)*(-1*speed)); 
    }
    void stop(){
        engine(0,0,0,0);  
    }
    void backward(double speed, int timeRunning, int timeStop){
        engine(speed, 0, -1, 0);
        sleep(timeRunning);
        stop();
        sleep(timeStop); 
    }
    void forward(double speed, int timeRunning, int timeStop){
        engine(speed, 0, 1, 0); 
        sleep(timeRunning);
        stop();
        sleep(timeStop);
    }
    void left(double speed, int timeRunning, int timeStop){
        engine(speed, -1, 0, 0); 
        sleep(timeRunning);
        stop();
        sleep(timeStop);
    }
    void right(double speed, int timeRunning, int timeStop){
        engine(speed, 1, 0, 0); 
        sleep(timeRunning);
        stop();
        sleep(timeStop);
    }
    void rotateLeft(double speed, int timeRunning, int timeStop){
        engine(speed, 0, 0, -1); 
        sleep(timeRunning);
        stop();
        sleep(timeStop);
    }
    void rotateRight(double speed, int timeRunning, int timeStop){
        engine(speed, 0, 0, 1); 
        sleep(timeRunning);
        stop();
        sleep(timeStop);
    }
    void carousselSpin(double power, int timeRunning, int timeStop){
        caroussel.setPower(power);
        sleep(timeRunning);
        caroussel.setPower(0);
        sleep(timeStop);
    }
    void intakeSpin(double power, int timeRunning, int timeStop){
        intake.setPower(power * -1);
        sleep(timeRunning);
        intake.setPower(0);
        sleep(timeStop);
    }void intakeArmSpin(double power, int timeRunning, int timeStop){
        intakeArm.setPower(power);
        sleep(timeRunning);
        intakeArm.setPower(0);
        sleep(timeStop);
    }private double getCurrentHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle);
    }}
