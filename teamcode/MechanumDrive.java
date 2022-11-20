package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MechanumDrive {
    private LinearOpMode myOpMode;
    Blinker control_Hub;
    Blinker expansion_hub;
    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;
    BNO055IMU imu;
    Orientation anglesHead;

    int prev_odo_leftY;
    int prev_odo_rightY;
    int prev_odo_centerX;

    double start_heading;
    double current_heading;

    public MechanumDrive(LinearOpMode opmode) {myOpMode = opmode;}

    public void init() {
        FrontL = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        FrontR = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        BackL = myOpMode.hardwareMap.get(DcMotor.class, "leftRear");
        BackR = myOpMode.hardwareMap.get(DcMotor.class, "rightRear");

        reset_heading();

        reset_odometry();

    }

    public void FieldCentric(double speed) {
        double theta = getCurrentHeading()+(3.1415/2);
        double FWD = (myOpMode.gamepad1.left_stick_x*Math.sin(theta) + myOpMode.gamepad1.left_stick_y*Math.cos(theta));
        double STR = (myOpMode.gamepad1.left_stick_x*Math.cos(theta) - myOpMode.gamepad1.left_stick_y*Math.sin(theta));
        double ROT = myOpMode.gamepad1.right_stick_x;
        speed = speed * -1;
        FrontL.setPower((FWD+STR+ROT)*(speed));
        FrontR.setPower((FWD-STR+ROT)*(speed));
        BackL.setPower(-1*(FWD-STR-ROT)*(speed));
        BackR.setPower(-1*(FWD+STR-ROT)*(speed));
        if (myOpMode.gamepad1.right_trigger > 0 && myOpMode.gamepad1.left_trigger > 0) {
            reset_heading();
        }
    }

    public double getCurrentHeading() {
        anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (anglesHead.firstAngle);
    }

    public void Forward(double speed) {
            FrontL.setPower(-1*speed);
            FrontR.setPower(speed);
            BackL.setPower(-1*speed);
            BackR.setPower(speed);
    }

    public void Backward(double speed) {
            Forward(speed * -1);
    }

    public void Left(double speed) {
            FrontL.setPower(speed * -1);
            FrontR.setPower(speed);
            BackL.setPower(speed);
            BackR.setPower(speed * -1);
    }

    public void Right(double speed) {
        Left(speed * -1);
    }

    public void RotateLeft(double speed) {
//        start_heading = getCurrentHeading();
//        degrees = degrees * 180/Math.PI;
//        while (current_heading < start_heading - degrees) {
            FrontL.setPower(speed * -1);
            FrontR.setPower(speed);
            BackL.setPower(speed * -1);
            BackR.setPower(speed);

//        }
//        Stop();
    }

    public void RotateRight(double speed) {
//        start_heading = getCurrentHeading();
//        degrees = degrees * 180/Math.PI;
//        while (current_heading < start_heading + degrees) {
            RotateLeft(speed * -1);
//        }
//        Stop();
    }

    public void Stop() {
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
    }

    public double get_odometry_x_In() {
        int current_odo_x = FrontL.getCurrentPosition() - prev_odo_centerX;
        int distance = current_odo_x / 1000;
        return Math.abs(distance);
    }

    public double get_odometry_y_In() {
        int current_odo_y1 = FrontR.getCurrentPosition() - prev_odo_leftY;
        int current_odo_y2 = BackL.getCurrentPosition() - prev_odo_rightY;
        myOpMode.telemetry.addData("Odo1", current_odo_y1);
        myOpMode.telemetry.addData("Odo2", current_odo_y2);
        int distance = (current_odo_y1 + current_odo_y2) / (2*1000);
        return Math.abs(distance);
    }

    public void reset_heading() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
    }

    public void reset_odometry() {
        prev_odo_leftY = FrontR.getCurrentPosition();
        prev_odo_rightY = BackL.getCurrentPosition();
        prev_odo_centerX = FrontL.getCurrentPosition();
    }


}
