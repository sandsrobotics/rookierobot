package org.firstinspires.ftc.teamcode; //    this is telling the robot what data you are useing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // // this gives the robot  //          ///////////// ///////////////////
// gives the robot info on your stuff ;)

@TeleOp

public class SixWheelArm extends LinearOpMode {
    private DcMotor leftMotor;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange = null;
    private Servo elbowServo;
    private Servo shoulderServo;
    private Servo clawServo;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");
        shoulderServo = hardwareMap.get(Servo.class, "shoulderservo");
        elbowServo = hardwareMap.get(Servo.class, "elbowservo");
        clawServo = hardwareMap.get(Servo.class, "clawservo");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double tgtPower2 = 0;
        while (opModeIsActive()) {

            tgtPower = this.gamepad1.left_stick_y;
            leftMotor.setPower(tgtPower);
            tgtPower2 = -this.gamepad1.right_stick_y;
            rightMotor.setPower(tgtPower2);

            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Left Motor Power", leftMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Shoulder Servo port 1", shoulderServo.getPosition());
            telemetry.addData("Elbow servo port 2", elbowServo.getPosition());
            telemetry.addData("Claw servo port 4", clawServo.getPosition());
            //telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            telemetry.update();

            // if (digitalTouch.getState() == false) {
            //       telemetry.addData("Button", "PRESSED");
            // } else {
            //    telemetry.addData("Button", "NOT PRESSED");


            // Raise arm at robot base "shoulder"
            if (gamepad1.y) {
                shoulderServo.setPosition(.5);
            } else if (gamepad1.x || gamepad1.b) {
                shoulderServo.setPosition(.75);
            } else if (gamepad1.a) {
                shoulderServo.setPosition(1);
            }

            // Raise arm at arm joint "elbow
            if (gamepad1.dpad_up) {
                elbowServo.setPosition(0);
            } else if (gamepad1.dpad_right || gamepad1.dpad_left) {
                elbowServo.setPosition(.5);
            } else if (gamepad1.dpad_down) {
                elbowServo.setPosition(1);
            }

            // Open and close claw
            if (gamepad1.right_trigger == 1.0 ) {
                clawServo.setPosition(0);
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                clawServo.setPosition(.5);
            } else if (gamepad1.left_trigger == 1.0) {
                clawServo.setPosition(1);


            }
        }
    }
}
