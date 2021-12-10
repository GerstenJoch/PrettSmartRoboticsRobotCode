package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
@TeleOp

public class FieldCentricTest extends OpMode {
    /* Declare OpMode members. */
    private Blinker control_Hub__2_;
    private Blinker expansion_Hub__7_;
    private DcMotor frontleft;
    //private Gyroscope imu_1;
    //private Gyroscope imu;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor intake;
    private DcMotor intakeArm;
    private DcMotor caroussel;
    private TouchSensor magTop;
    double powerFL;
    double powerFR;
    double powerBR;
    double powerBL;
    double FWD;
    double STR;
    double ROT;
    double speed;
    double intakePower;
    double intakeArmPower;
    double speedArm;
    double theta;
    String speedTelemetry;
    String speedArmTelemetry;
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
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");
        backleft = hardwareMap.get(DcMotor.class, "rearleft");
        backright = hardwareMap.get(DcMotor.class, "rearright");
        magTop = hardwareMap.get(TouchSensor.class, "magTop");
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
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        //Driving Mechanism
        drivingMechanism();
        //Caroussel Mechanism (double is for power Caroussel when on)
        caroussel(0.3);
        //IntakeArm Mechanism (double is for power IntakeArm when on)
        intakeArm(0.8);       
        //Intake Mechanism (double is for power Intake when on)
        intake(1);
        //Adding data to screen
        telemetryOutput();
        telemetry.update();
    }
    @Override
    public void stop() {

    }
    //Methods for loop()


    void drivingMechanism(){
        //powerBL = (gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerFL = (gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerBR = ((-gamepad1.left_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerFR = ((-gamepad1.left_stick_y) - gamepad1.left_stick_x) - gamepad1.right_stick_x;
        theta = getCurrentHeading()+(3.1415/2);
        FWD = (gamepad1.left_stick_x*Math.sin(theta) + gamepad1.left_stick_y*Math.cos(theta));
        STR = (gamepad1.left_stick_x*Math.cos(theta) - gamepad1.left_stick_y*Math.sin(theta));
        ROT = gamepad1.right_stick_x/2;
        speed = -1;
        if (gamepad1.left_bumper){
            speed = speed/2;
        }else if (gamepad1.right_bumper){
            speed = speed/4;
        }else{
            speed = -1;
        }
        frontleft.setPower((FWD+STR+ROT)*speed);
        frontright.setPower((FWD-STR+ROT)*speed);
        backleft.setPower((-1*FWD-STR+ROT)*speed);
        backright.setPower((-1*FWD+STR+ROT)*speed);
    }
    void caroussel(double carousselPower){
        if (gamepad2.y){
            caroussel.setPower(carousselPower);
        }else{
            caroussel.setPower(0);
        }
    }
    void telemetryOutput(){
        //if statements for variables

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
        if (speed == -0.5){
            speedTelemetry = "Half Speed";
        }else if (speed == -0.25){
            speedTelemetry = "Quarter Speed";
        } else {
            speedTelemetry = "Full Speed";
        }if (speedArm == -0.2){
            speedArmTelemetry = "Quarter Speed";
        }else if (speedArm == -0.4){
            speedArmTelemetry = "Half Speed";
        }else {
            speedArmTelemetry = "Full Speed";
        }

        //Ads data to Telemetry
        telemetry.addData("FWD ", FWD);
        telemetry.addData("STR ", STR);
        telemetry.addData("ROT ", ROT);
        telemetry.addData("RobotSpeed ", speedTelemetry);
        telemetry.addData("Caroussel ", carousselTelemetry);
        //telemetry.addData("Intake", intakeTelemetry);
        telemetry.addData("IntakeArmSpeed", speedArmTelemetry);
        telemetry.addData("Magnetic ", gamepad2.left_stick_y);
    }
    void intake(double intakePower){
        if (gamepad2.b){
            intake.setPower(intakePower);
        }else{
            intake.setPower(0);
        }
        if (gamepad2.a){
            intake.setPower(-1*(intakePower));
        }else{
            intake.setPower(0);
        }
    }
    void intakeArm(double speedArmInput){
        if (gamepad2.left_bumper){
            speedArm = -1*(speedArmInput/2);
        }else if (gamepad2.right_bumper && gamepad2.left_bumper){
            speedArm = -1*(speedArmInput/4);
        }else{
            speedArm = -1*speedArmInput;
        }
        if(magTop.isPressed() && gamepad2.left_stick_y<=0){
            intakeArm.setPower(0);
        }else {
            intakeArm.setPower(speedArm*gamepad2.left_stick_y);
        }   
    }

    //Gets the current heading of the Robot
    private double getCurrentHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle);
    }
}
