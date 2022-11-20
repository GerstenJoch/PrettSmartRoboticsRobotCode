package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Driver extends OpMode {
    /* Declare OpMode members. */
    private DcMotor frontleft;
    //private Gyroscope imu_1;
    //private Gyroscope imu;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor arm;
    private DcMotor caroussel;
    private Servo gripperArmBlock;
    private Servo rotateIntake;
    private DigitalChannel gripperRedLED;
    private DigitalChannel gripperGreenLED;
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
    int x = 0;
    long armTime;
    long armTimeElapsed;
    long gripperArmTime;
    long gripperArmTimeElapsed;
    long intakeTimeElapsed;
    long intakeTime;
    int odo1;
    int beginPos;
    double theta;
    double startAngle;
    double phi;
    String speedTelemetry;
    String speedArmTelemetry;
    String intakeTelemetry;
    String intakeArmTelemetry;
    String carousselTelemetry;
    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation anglesHead;
    Orientation anglesPitch;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        frontleft = hardwareMap.get(DcMotor.class, "leftFront");
        frontright = hardwareMap.get(DcMotor.class, "rightFront");
        arm = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(Servo.class, "intake");
        rotateIntake = hardwareMap.get(Servo.class, "rotateIntake");
        //imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //caroussel = hardwareMap.get(DcMotor.class, "caroussel");
        //rolmaat = hardwareMap.get(DcMotor.class, "rolmaat");
        backleft = hardwareMap.get(DcMotor.class, "leftRear");
        backright = hardwareMap.get(DcMotor.class, "rightRear");
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
        //startAngle = getCurrentPitch();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //telemetry.addData("IMU calibration ", "IN PROGRESS...");
        //telemetry.update();
        //intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //beginPos = caroussel.getCurrentPosition();
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        //odo1 = caroussel.getCurrentPosition() - beginPos; //caroussel is odo1 encoder
        //Driving Mechanism
        drivingMechanism(.75);
        
        //Lift
        if (gamepad2.a) {
            intake.setPosition(0.1 );
        } else if (gamepad2.b) {
            intake.setPosition(.6);
        }
        if (gamepad2.y) {
            rotateIntake.setPosition(1);
        } else if (gamepad2.x) {
            rotateIntake.setPosition(0);
        }
        if (gamepad2.right_stick_y != 0) {
            arm.setPower(1.5 * gamepad2.right_stick_y);
        } else {
            arm.setPower(0);
        }



        //Caroussel Mechanism (double is for power Caroussel when on)
        //caroussel(0.4);
        reset();
        //IntakeArm Mechanism (double is for power IntakeArm when on)
        //intakeArm(1);
        //Intake Mechanism (double is for power Intake when on)
        //intake();
        //gripperArm(0.5);
        //autoBalance();
        //Rolmaat Mechanism (double is for power Rolmaat when on)
        //rolMaatPower(1);
        //Adding data to screen
        //telemetryOutput();
        //telemetry.update();
    }
    @Override
    public void stop() {
    }
    //Functions for loop()


    void drivingMechanism(double speed){
        //powerBL = (gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerFL = (gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerBR = ((-gamepad1.left_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //powerFR = ((-gamepad1.left_stick_y) - gamepad1.left_stick_x) - gamepad1.right_stick_x;
        // theta = getCurrentHeading()+(3.1415/2);
        // FWD = (gamepad1.left_stick_x*Math.sin(theta) + gamepad1.left_stick_y*Math.cos(theta));
        // STR = (gamepad1.left_stick_x*Math.cos(theta) - gamepad1.left_stick_y*Math.sin(theta));
        // ROT = gamepad1.right_stick_x/2;
        engine(speed, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
//    void caroussel(double speed){
//        if (gamepad2.y){
//            caroussel.setPower(speed);
//        }else if (gamepad2.x){
//            caroussel.setPower(speed*-1);
//        }else{
//            caroussel.setPower(0);
//
//        }
//    }
    // void rolMaatPower(double speed){
    //     if(gamepad2.a){
    //         rolmaat.setPower(speed);

    //     }else{
    //         rolmaat.setPower(0);
    //     }
    //     if (gamepad2.b){
    //         rolmaat.setPower(speed * -1);
    //     }else {
    //         rolmaat.setPower(0);

    //     }
    // }
    void reset(){
        if (gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;
            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu.initialize(parameters);
        }
    }
//    void intake(){
//        long currentTime;
//        if (gamepad2.right_bumper && intakeTimeElapsed > 550){
//            double intakePos = intake.getPosition();
//            if ((int)intakePos == 0){
//                intake.setPosition(1);
//                gripperRedLED.setState(true);
//                gripperGreenLED.setState(false);
//            } if ((int)intakePos == 1){
//                intake.setPosition(0);
//                gripperRedLED.setState(false);
//                gripperGreenLED.setState(true);
//            }
//            intakeTime = System.currentTimeMillis();
//            intakeTimeElapsed = 0;
//        }
//        currentTime = System.currentTimeMillis();
//        intakeTimeElapsed = currentTime - intakeTime;
//    }
//    void intakeArm(double speed){
//        if(gamepad2.left_stick_y==0){
//            intakeArm.setPower(0);
//            intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }else {
//            intakeArm.setPower(-1* speed*gamepad2.left_stick_y);
//        }
//
//    }
//    void gripperArm(double speed){
//        if(gamepad2.right_stick_y==0){
//            gripperIntakeArm.setPower(0);
//        }else {
//            gripperIntakeArm.setPower(-1 * speed*gamepad2.right_stick_y);
//
//        }
//    }
    // void autoBalance(){
    //     phi = getCurrentPitch();
    //     if (gamepad2.left_bumper){
    //         if (phi < (startAngle-2) && phi > (startAngle+2)){
    //             double dPhi = phi - startAngle;
    //             telemetry.addData("dPhi", dPhi);
    //             if (phi < 0){
    //                 engine((1/90) * dPhi, -1, 0, 0);
    //                 telemetry.addData("Left", dPhi);
    //             } if (phi > 0){
    //                 engine((1/90) * dPhi, 1,0 ,0);
    //                 telemetry.addData("Right", dPhi);
    //             }
    //             }
    //         }
    //     else {
    //         return;
    //     }
    // }
//    void telemetryOutput(){
//        //if statements for variables
//
//        /*if (carousselPower == 0){
//            carousselTelemetry = "isNotRunning";
//        }else {
//            carousselTelemetry = "isRunning";
//        }*/ if (intakeArmPower == 0){
//            intakeArmTelemetry = "isNotRunning";
//        }else {
//            intakeArmTelemetry = "isRunning";
//        } /*if (intakePower == 0){
//            intakeTelemetry = "isNotRunning";
//        }else {
//            intakeTelemetry = "isRunning";}*/
//        if (speed == -0.5){
//            speedTelemetry = "Half Speed";
//        }else if (speed == -0.25){
//            speedTelemetry = "Quarter Speed";
//        } else {
//            speedTelemetry = "Full Speed";
//        }if (speedArm == -0.2){
//            speedArmTelemetry = "Quarter Speed";
//        }else if (speedArm == -0.4){
//            speedArmTelemetry = "Half Speed";
//        }else {
//            speedArmTelemetry = "Full Speed";
//        }

//        //Ads data to Telemetry
//        telemetry.addData("Odo1", odo1);
//        telemetry.addData("FWD ", FWD);
//        telemetry.addData("STR ", STR);
//        telemetry.addData("ROT ", ROT);
//        telemetry.addData("RobotSpeed ", speedTelemetry);
//        //telemetry.addData("Intake", intakeTelemetry);
//        telemetry.addData("IntakeArmSpeed", speedArmTelemetry);
//        telemetry.addData("Magnetic ", gamepad2.left_stick_y);
//        telemetry.addData("Current Y pos", gamepad2.left_stick_y);
//        telemetry.addData("Intake Time Elapsed",intakeTimeElapsed);
//        telemetry.addData("Phi", phi);
//    }
    void engine(double speed, double x, double y, double r){
        theta = getCurrentHeading()+(3.1415/2);
        FWD = (x*Math.sin(theta) + y*Math.cos(theta));
        STR = (x*Math.cos(theta) - y*Math.sin(theta));
        ROT = r;
        speed = speed * -1;
        frontleft.setPower((FWD+STR+ROT)*(speed));
        frontright.setPower((FWD-STR+ROT)*(speed));
        backleft.setPower(-1*(FWD-STR-ROT)*(speed));
        backright.setPower(-1*(FWD+STR-ROT)*(speed));
    }
    //Gets the current heading of the Robot
    private double getCurrentHeading(){
        anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (anglesHead.firstAngle);
    }

    // private double getCurrentPitch(){
    //     anglesPitch = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
    //     return (anglesPitch.firstAngle);
    // }
}


