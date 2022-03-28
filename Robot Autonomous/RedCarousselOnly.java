package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class RedCarousselOnly extends LinearOpMode {
    /* Declare OpMode members. */
    private Blinker control_Hub__2_;
    private Blinker expansion_Hub__7_;
    private DcMotor frontleft;
    //private Gyroscope imu_1;
    //private Gyroscope imu;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor intakeArm;
    private DcMotor caroussel;
    private Servo gripperArmBlock;
    //private TouchSensor limit;
    private Servo intake;
    // private DcMotor rolmaat;
    private DcMotor gripperIntakeArm;
    double powerFL;
    double powerFR;
    double powerBR;
    double powerBL;
    double FWD;
    double STR;
    double ROT;
    double speed;
    double speedArm;
    double intakeArmPower;
    int beginArmPos;
    int armPos;
    int deltaArmPos;
    long armTime;
    long armTimeElapsed;
    long gripperArmTime;
    long gripperArmTimeElapsed;
    long intakeTimeElapsed;
    long intakeTime;
    double theta;
    String speedTelemetry;
    String speedArmTelemetry;
    String intakeTelemetry;
    String intakeArmTelemetry;
    String carousselTelemetry;
    // State used for updating telemetry
    Orientation angles;
    int x = 1;
    BNO055IMU imu;
    boolean isDuckDetected = false;
    
    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        control_Hub__2_ = hardwareMap.get(Blinker.class, "Control Hub (2)");
        expansion_Hub__7_ = hardwareMap.get(Blinker.class, "Expansion Hub (7)");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        //imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        caroussel = hardwareMap.get(DcMotor.class, "caroussel");
        intake = hardwareMap.get(Servo.class, "intake");
        //rolmaat = hardwareMap.get(DcMotor.class, "rolmaat");
        gripperArmBlock = hardwareMap.get(Servo.class, "gripperArmBlock");
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");
        gripperIntakeArm = hardwareMap.get(DcMotor.class, "gripperIntakeArm");
        backleft = hardwareMap.get(DcMotor.class, "rearleft");
        backright = hardwareMap.get(DcMotor.class, "rearright");
        //limit = hardwareMap.get(TouchSensor.class, "limit");
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
        intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("IMU calibration ", "IN PROGRESS...");
        telemetry.update();
        

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        intake.setPosition(1);
        waitForStart();
        if (opModeIsActive()){
            intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeTime = System.currentTimeMillis();
        intake.setPosition(0);
        try {
        TimeUnit.MILLISECONDS.sleep(1000);
        } catch (InterruptedException e){}
        beginArmPos = intakeArm.getCurrentPosition();
        deltaArmPos = 0;
        while (deltaArmPos < 1250){
            intakeArm.setPower(0.7);
            armPos = intakeArm.getCurrentPosition();
            deltaArmPos = armPos - beginArmPos;
            
        } intakeArm.setPower(0); 
        
        gripperArmTime = System.currentTimeMillis();
        gripperArmTimeElapsed = 0;
        while (gripperArmTimeElapsed < 500){
            gripperIntakeArm.setPower(-0.8);
            long currentTime = System.currentTimeMillis();
            gripperArmTimeElapsed = currentTime - gripperArmTime;
        } gripperIntakeArm.setPower(0);
        try {
        TimeUnit.MILLISECONDS.sleep(200);
        } catch (InterruptedException e){}
        armTime = System.currentTimeMillis();
        armTimeElapsed = 0;
        while (armTimeElapsed < 200){
          intakeArm.setPower(-0.2);
          long currentTime = System.currentTimeMillis();
          armTimeElapsed = currentTime - armTime;
        } intakeArm.setPower(0);
        gripperArmBlock.setPosition(-1);
        

}
        while (opModeIsActive()){
            intake.setPosition(1);
            right(0.5,200,100);
            rotateLeft(0.4,900,100);
            left(0.5,250,100);
            intake.setPosition(0);
            rotateRight(0.4,900,100);
            rotateRight(0.4,900,100);
            left(0.4,300,100);
            carousselSpin(-0.3, 3500,100);
            forward(0.5,600,100);
            rotateLeft(0.4,900,100);
            left(0.7,700,100);
            intake.setPosition(1);
            sleep(30000);
        }
    }
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
    void stopBot(){
        engine(0,0,0,0);  
    }
    void backward(double speed, int timeRunning, int timeStop){
        engine(speed, 0, 1, 0);
        sleep(timeRunning);
        stopBot();
        sleep(timeStop); 
    }
    void forward(double speed, int timeRunning, int timeStop){
        engine(speed, 0, -1, 0); 
        sleep(timeRunning);
        stopBot();
        sleep(timeStop);
    }
    void left(double speed, int timeRunning, int timeStop){
        engine(speed, -1, 0, 0); 
        sleep(timeRunning);
        stopBot();
        sleep(timeStop);
    }
    void right(double speed, int timeRunning, int timeStop){
        engine(speed, 1, 0, 0); 
        sleep(timeRunning);
        stopBot();
        sleep(timeStop);
    }
    void rotateLeft(double speed, int timeRunning, int timeStop){
        engine(speed, 0, 0, -1); 
        sleep(timeRunning);
        stopBot();
        sleep(timeStop);
    }
    void rotateRight(double speed, int timeRunning, int timeStop){
        engine(speed, 0, 0, 1); 
        sleep(timeRunning);
        stopBot();
        sleep(timeStop);
    }
    void carousselSpin(double power, int timeRunning, int timeStop){
        caroussel.setPower(power);
        sleep(timeRunning);
        caroussel.setPower(0);
        sleep(timeStop);
    }
    // void intakeSpin(double power, int timeRunning, int timeStop){
    //     intake.setPower(power * -1);
    //     sleep(timeRunning);
    //     intake.setPower(0);
    //     sleep(timeStop);
    void intakeArm(double power, int timeRunning, int timeStop){
        intakeArm.setPower(power);
        sleep(timeRunning);
        intakeArm.setPower(0);
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(timeStop);
    }
private double getCurrentHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle);
    }}
