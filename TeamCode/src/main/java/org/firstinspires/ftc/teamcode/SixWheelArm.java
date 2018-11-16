package org.firstinspires.ftc.teamcode; //    this is telling the robot what data you are useing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// gives the robot info on your stuff ;)

@TeleOp

public class SixWheelArm extends LinearOpMode {
    private DcMotor leftMotor;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange = null;
    private Servo elbowServo;
    private Servo shoulderServo;
    private Servo clawServo;
    private Servo wristServo;
    private DcMotor rightMotor;
    private DcMotor lifter;
    private ColorSensor sensorColor;


    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");
        shoulderServo = hardwareMap.get(Servo.class, "shoulderservo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowservo");
        clawServo = hardwareMap.get(Servo.class, "clawservo");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double tgtPower2 = 0;
        double tgtPower3 = 0;
        while (opModeIsActive()) {


            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Left Motor Power", leftMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Shoulder Servo port 1", shoulderServo.getPosition());
            telemetry.addData("Elbow servo port 2", elbowServo.getPosition());
            telemetry.addData("Claw servo port 4", clawServo.getPosition());
            telemetry.addData("wrist servo port ", wristServo.getPosition());
            telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.update();

            // if (digitalTouch.getState() == false) {
            //       telemetry.addData("Button", "PRESSED");
            // } else {
            //    telemetry.addData("Button", "NOT PRESSED");

// controler 1
            // Raise arm at robot base "shoulder"
            if (gamepad1.right_stick_y > 1 ){
                shoulderServo.setPosition(shoulderServo.getPosition() + 0.03f);
            }
            if (gamepad1.right_stick_y > -1 ) {
                shoulderServo.setPosition(shoulderServo.getPosition() - 0.03f);
            }

            // Raise arm at arm joint elbow
            if (gamepad1.dpad_left) {
                elbowServo.setPosition(elbowServo.getPosition() + 0.01);
            } else if (gamepad1.dpad_right) {
                elbowServo.setPosition(elbowServo.getPosition() - 0.01);
            }

            // Open and close claw
            if (gamepad1.dpad_up) {
                clawServo.setPosition(clawServo.getPosition() + 0.01);

            } else if (gamepad1.dpad_down) {
                clawServo.setPosition(clawServo.getPosition() - 0.01);
            }

            // dunk the wrist bro
            if (gamepad1.x) {
                wristServo.setPosition(wristServo.getPosition() + 0.01);
            } else if (gamepad1.y) {
                wristServo.setPosition(wristServo.getPosition() - 0.01);
            }


// controler 2
            //driving
            tgtPower = this.gamepad2.left_stick_y;
            leftMotor.setPower(tgtPower);
            tgtPower2 = -this.gamepad2.right_stick_y;
            rightMotor.setPower(tgtPower2);

            // lift up and down
            if (gamepad2.right_trigger == 1) {
                lifter.setPower(.75);
            } else if (gamepad2.left_trigger == 1) {
                lifter.setPower(-.75);
            }
            if (gamepad2.dpad_up) {
                lifter.setPower(0);
            }
        }
    }

}