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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Stack;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//voor de webcam:
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//voor de GYRO:
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class AutonomousRed01 extends LinearOpMode {
    private Blinker control_Hub__2_;
    private Blinker expansion_Hub__7_;
    private HardwareDevice webcam;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor shooter;
    private DcMotor rearleft;
    private DcMotor rearright;
    private ColorSensor sensorLeft;
    private ColorSensor sensorRight;
    private Servo shooterServo;

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;

    // webcam
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "Ae5Xi6v/////AAABmQvuQQ6zX0SxnG46H8JGXBY1dCZuS1CJP03cqN9YTTp1Jdc7iPwt+wb7CK6X2Jk5HxZ1pVMSqHRId2Awv2xIS/B3TVYfjXzBeUCJcOe55oDU8DjQJT4gmpbTRtPDCJMW1phw7P6hcGszxcCQgNg2RFjF2tZGsMCbt1w77p+dvqYCcjJJub+UbPZjK0GIotutNhOqJxFs5EDOHs9SJXXnCOloPE4bKr8CBppvnzCoitRb4/FD6u8bI7gwYioBjlyQfMLeFKYTUm8048JVZz5H9vRYrS/FSQzD0Wd3/dYk7EvB+VfXUsnel9kZRgnOTCm9rmp+u3I9YW5cZfFe+x/jm1sOV2bG7gPCBSQocDvosiZo";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private String stackDetected;
    int stack;
    boolean debug = false;

    @Override
    public void runOpMode() {
        control_Hub__2_ = hardwareMap.get(Blinker.class, "Control Hub (2)");
        expansion_Hub__7_ = hardwareMap.get(Blinker.class, "Expansion Hub (7)");
        webcam = hardwareMap.get(HardwareDevice.class, "Webcam");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        rearleft = hardwareMap.get(DcMotor.class, "rearleft");
        rearright = hardwareMap.get(DcMotor.class, "rearright");
        sensorLeft = hardwareMap.get(ColorSensor.class, "sensorLeft");
        sensorRight = hardwareMap.get(ColorSensor.class, "sensorRight");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        double driveStraightPower = 0.4;
        double turnPower = 0.5;


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

        //checks every 50 milliseconds if the gyro is calibrated.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("IMU calibration ", "READY");

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);

        }

        telemetry.addLine("Waiting for start");
        telemetry.update();



        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0 ) {
                        // empty list.  no objects recognized.
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
                        modeA();
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                modeB();
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                modeC();
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                            }
                        }}

                    telemetry.update();
                }
            }


        }
    }

    void modeA(){
        telemetry.addLine("mode A");
        telemetry.addData("Rood rechts", sensorRight.red());
        telemetry.addData("Blauw rechts", sensorRight.blue());
        telemetry.addData("Rood links", sensorLeft.red());
        telemetry.addData("Blauw links", sensorLeft.blue());
        telemetry.update();
        if(debug){stoppen();}
        moveStraightToColor(0.5,"RODE lijn", "right");
        turnClockwiseToHeading(0.6, 45);
        moveStraightForDistance(0.4,600);
        moveStraightForDistance(-0.4, 1500);
        turnAntiClockwiseToHeading(0.6, 10);
        //moveStraightToColor(-0.4, "WITTE LIJN", "left");
        testShooter(-0.65);
        //moveStraightToColor(0.4,"WITTE lijn", "right");
        moveStraightForDistance(0.4, 800);
        sleep(30000);
    }

    void modeB(){
        telemetry.addLine("mode B");
        telemetry.addData("Rood rechts", sensorRight.red());
        telemetry.addData("Blauw rechts", sensorRight.blue());
        telemetry.addData("Rood links", sensorLeft.red());
        telemetry.addData("Blauw links", sensorLeft.blue());
        telemetry.update();
        if(debug){stoppen();}
        moveStraightToColor(0.4,"RODE lijn", "left");
        turnAntiClockwiseToHeading(0.6, -45);
        moveStraightForDistance(0.5,500);
        turnAntiClockwiseToHeading(0.6, -45);
        moveStraightForDistance(-0.5, 500);
        turnClockwiseToHeading(0.5, -5);
        //moveStraightToColor(-0.4,"WITTE lijn", "left");
        moveStraightForDistance(-0.4, 1000);
        turnAntiClockwiseToHeading(0.6, -10);
        testShooter(-0.6);
        moveStraightForDistance(0.6, 400);
        //moveStraightToColor(0.4,"WITTE lijn", "left");
        sleep(30000);
    }

    void modeC(){
        telemetry.addLine("mode C");
        telemetry.addData("Rood rechts", sensorRight.red());
        telemetry.addData("Blauw rechts", sensorRight.blue());
        telemetry.addData("Rood links", sensorLeft.red());
        telemetry.addData("Blauw links", sensorLeft.blue());
        telemetry.update();
        if(debug){stoppen();}
        moveStraightToColor(0.4,"RODE lijn", "right");
        moveStraightToColor(0.4,"RODE lijn", "left");
        moveStraightToColor(0.4,"RODE lijn", "right");
        turnClockwiseToHeading(0.5, 45);
        moveStraightForDistance(0.4,1000);
        moveStraightForDistance(-0.4, 1500);//1000
        turnAntiClockwiseToHeading(0.5, 0);
        //moveStraightToColor(-0.4,"WITTE lijn", "right");
        moveStraightForDistance(-0.4, 1500);//1000
        turnClockwiseToHeading(0.5, 2.5);
        testShooter(-0.65);
        //moveStraightToColor(0.4,"WITTE lijn", "right");
        moveStraightForDistance(0.4, 800);
        sleep(30000);
    }

    void stoppen(){
        sleep(30000);
    }

    void shoot(){
        moveStraightToColor(-0.3, "WITTE lijn", "right");
        testShooter(-0.6);
        moveStraightToColor(0.3, "WITTE lijn", "right");
    }

    void testShooter(double power){
        shooterServo.setPosition(0);
        shooter.setPower(power);
        sleep(3000);

        shooterServo.setPosition(1);
        sleep(200);
        shooterServo.setPosition(0);
        sleep(500);

        shooterServo.setPosition(1);
        sleep(300);
        shooterServo.setPosition(0);
        sleep(500);

        shooterServo.setPosition(1);
        sleep(300);
        shooterServo.setPosition(0);
        sleep(500);

        shooter.setPower(0);
    }


    void moveStraightToColor(double power, String targetColor, String leftRight){
        setMotorPowers(power*-1,power*-1,power,power);
        if(leftRight == "left") {
            while (targetColor != colorToString(sensorLeft.red()*95,sensorLeft.blue())){
                sleep(50);
            }
        } else {
            while (targetColor != colorToString(sensorRight.red()*105,sensorRight.blue())){
                sleep(50);
            }
        }
        setMotorPowers(0,0,0,0);
    }

    void moveStraightForDistance(double power, long time){
        setMotorPowers(power*-1,power*-1,power,power);
        //while (targetColor != colorToString(sensorLeft.red(),sensorLeft.blue())){
        //    sleep(50);
        //} hier gaan we de encoders invoegen
        sleep(time);
        setMotorPowers(0,0,0,0);

    }

    private double getCurrentHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle * -1);
    }

    void turnClockwiseToHeading(double power, double heading){
        power*=-1;
        setMotorPowers(power,power,power,power);
        while (getCurrentHeading() < heading ){
            sleep(50);
            getCurrentHeading();
        }
        setMotorPowers(0,0,0,0);
    }

    void turnAntiClockwiseToHeading(double power, double heading){
        setMotorPowers(power,power,power,power);
        while (getCurrentHeading() > heading ){
            sleep(50);
            getCurrentHeading();
        }
        setMotorPowers(0,0,0,0);
    }

    java.lang.String colorToString(int red, int blue){

        String MyColor = "GRIJZE mat";
        if(red>2500 && blue>4000){
            MyColor = "WITTE lijn";
        }
        if(red>1200 && blue<400){
            MyColor = "RODE lijn";
        }
        if(red<100000 && blue>100000){
            MyColor = "BLAUWE lijn";
        }
        return MyColor;
    }

    void setMotorPowers(double frontleftPower, double rearleftPower, double frontrightPower, double rearrightPower){
        frontleft.setPower(frontleftPower);
        rearleft.setPower(rearleftPower);
        frontright.setPower(frontrightPower);
        rearright.setPower(rearrightPower);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


}