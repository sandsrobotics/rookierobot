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

public class SixWheelArm {
    private DcMotor leftMotor;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange = null;
    private Servo elbowServo;
    private Servo shoulderServo;
    private Servo clawServo;
    private Servo wristServo;
    private DcMotor rightMotor;
    private DcMotor lifter;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");
        shoulderServo = hardwareMap.get(Servo.class, "shoulderservo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowservo");
        clawServo = hardwareMap.get(Servo.class, "clawservo");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double tgtPower2 = 0;
        double tgtPower3 = 0;
        while(AutonomousModeActive) ;
        {

            tgtPower = this.gamepad1.left_stick_y;
            leftMotor.setPower(tgtPower);
            tgtPower2 = -this.gamepad1.right_stick_y;
            rightMotor.setPower(tgtPower2);


            // if (digitalTouch.getState() == false) {
            //       telemetry.addData("Button", "PRESSED");
            // } else {
            //    telemetry.addData("Button", "NOT PRESSED");


            // Raise arm at robot base "shoulder"
            if (gamepad1.x) {
                shoulderServo.setPosition(shoulderServo.getPosition() + 0.03f);
            }
        }
    }
}