package org.firstinspires.ftc.teamcode; //    this is telling the robot what data is being used

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //importation of data about the robot's hardware
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp // this makes the robot controlled by a controller

public class SixWheelArm extends LinearOpMode {   // addition of the hardware's software

    private DcMotor leftMotor;
    private DigitalChannel digitalTouch;
    private DigitalChannel digitalTouch2;
    private DistanceSensor sensorColorRange = null;
    //private CRServo elbowServo;
    private CRServo elbowServo;
    private Servo clawServo;
    private Servo wristServo;
    private DcMotor rightMotor;
    private DcMotor lifter;
    private ColorSensor sensorColor;
    private DcMotor shoulderServo;
    private CRServo clawServo2;
    private AnalogInput AngleSensor;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        elbowServo = hardwareMap.get(CRServo.class, "elbowservo");
        wristServo = hardwareMap.get(Servo.class, "wristservo");
        //elbowServo = hardwareMap.get(Servo.class, "elbowservo");
        clawServo = hardwareMap.get(Servo.class, "clawservo");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        shoulderServo = hardwareMap.get(DcMotor.class, "sholderServo");
        clawServo2 = hardwareMap.get(CRServo.class, "clawServo2");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch2 = hardwareMap.get(DigitalChannel.class, "digitalTouch2");
        digitalTouch2.setMode(DigitalChannel.Mode.INPUT);
        AngleSensor = hardwareMap.get(AnalogInput.class, "AngleSensor");
        shoulderServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;     // adding data
        double tgtPower2 = 0;
        double tgtPower3 = 0;
        double NewIdea = 0;
        double shoulder = 1;
        double Angle = 0;
        int T = 0;
        double TT = 0;
        double ex = 0;
        double PT = 0;

        while (opModeIsActive()) {

            Angle = (AngleSensor.getVoltage()) * 81; // change if you change voltige D/V (270 / 3.33)

            telemetry.addData("Target Power", tgtPower);  // report to the phone screen about what the robot is doing
            telemetry.addData("Left Motor Power", leftMotor.getPower());
            //telemetry.addData("Status", "Running");
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Shoulder Servo port 1", shoulderServo.getPower());
            telemetry.addData("Elbow servo port 2", elbowServo.getPower());
            telemetry.addData("Claw servo port 4", clawServo.getPosition());
            telemetry.addData("wrist servo port ", wristServo.getPosition());
            telemetry.addData("volts", AngleSensor.getVoltage());
            telemetry.addData("Angle", Angle);
            telemetry.addData("Encodeer", shoulderServo.getCurrentPosition());
            //telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            //telemetry.addData("Alpha", sensorColor.alpha());
            //telemetry.addData("Red  ", sensorColor.red());
            //telemetry.addData("Green", sensorColor.green());
            //telemetry.addData("Blue ", sensorColor.blue());


            telemetry.update();

            if (!digitalTouch.getState()) {
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }

// controler 1
            // Raise arm at robot base "shoulder"
            if (gamepad1.a) {
                if (shoulderServo.getCurrentPosition() > -1500) { // if the on bord button is pressed
                    shoulderServo.setPower(0);
                }// then do this
                else {  //if not pressed
                    shoulderServo.setPower(.5); // do this
                }
            }

            if (gamepad1.b) { // if b is pressed
                if (!digitalTouch.getState()) // if the on bord button is pressed
                    shoulderServo.setPower(0); // then do this
                else  //if not pressed
                    shoulderServo.setPower(-.5); // do this
            }

            if (!gamepad1.a && !gamepad1.b)
                shoulderServo.setPower(0);

            // Raise arm at arm joint elbow
            if (gamepad1.x) {
                T = 1;
                ex = 0;
                if (Angle > 250) { // if the on bord button is pressed
                    elbowServo.setPower(0);
                }// then do this
                else {  //if not pressed
                    elbowServo.setPower(-.5); // do this
                }
            }

            if (gamepad1.y) {
                T = 1;
                ex = 0;
                if (Angle < 20) { // if the on bord button is pressed
                    elbowServo.setPower(0);
                }// then do this
                else {  //if not pressed
                    elbowServo.setPower(.5); // do this
                }
            }
            // STAY STILL
            if (!gamepad1.x && !gamepad1.y) {
                if (T == 3) {
                    if ((TT) > Angle) {
                       PT = Angle - TT ;
                        elbowServo.setPower(-.05 + (PT/11));
                    } else if (TT < Angle) {

                        elbowServo.setPower(.05 + ex);

                    }
                } else {
                    TT = Angle;
                    T = 3;
                }
            }

            // Open and close claw
            if (gamepad1.right_trigger == 1) {
                clawServo.setPosition(clawServo.getPosition() + 0.1);

            } else if (gamepad1.left_trigger == 1) {
                clawServo.setPosition(clawServo.getPosition() - 0.1);

            } else if (gamepad1.left_trigger == 0 & gamepad1.right_trigger == 0) {
                clawServo.setPosition(0);
            }

            // Open and close claw2
            if (gamepad1.right_trigger == 1) {
                clawServo2.setPower(.7);

            } else if (gamepad1.left_trigger == 1) {
                clawServo2.setPower(-.7);

            } else if (gamepad1.left_trigger == 0 & gamepad1.right_trigger == 0) {
                clawServo2.setPower(0);
            }

            // dunk the wrist bro
            if (gamepad1.dpad_left) {
                wristServo.setPosition(wristServo.getPosition() + 0.02);
            } else if (gamepad1.dpad_right) {
                wristServo.setPosition(wristServo.getPosition() - 0.02);
            }

// controler 2
            //driving
            if (gamepad2.a)
                NewIdea = 1;
            if (gamepad2.b)
                NewIdea = .5;
            tgtPower = this.gamepad2.left_stick_y;
            leftMotor.setPower(tgtPower * NewIdea);
            tgtPower2 = -this.gamepad2.right_stick_y;
            rightMotor.setPower(tgtPower2 * NewIdea);


            // lift up and down
            if (gamepad2.right_trigger == 1) {
                lifter.setPower(0.5);
            }
            if (gamepad2.left_trigger == 1) {
                lifter.setPower(-0.5);
            }
            {
            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
                lifter.setPower(0);

            }
            // dump position
            if (gamepad2.dpad_up) {
                wristServo.setPosition(0.58);

                shoulderServo.setTargetPosition(-1500);
                shoulderServo.setPower(.5);
                TT = 100;


                }


            //get position
            if (gamepad2.dpad_down) {
                wristServo.setPosition(0.47); //\\//\\ NOTE ANTONIO SET SHOULDER POWER TO FIX BUG
                shoulderServo.setTargetPosition(-3000);
                shoulderServo.setPower(.5);
                TT = 30;

            }

            }
        }

    }



    // testing hardware


