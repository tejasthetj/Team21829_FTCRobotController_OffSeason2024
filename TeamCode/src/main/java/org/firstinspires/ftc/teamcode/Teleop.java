package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


    @TeleOp(name = "Teleop", group = "robot")

    public class Teleop extends LinearOpMode {

        private DcMotor leftFrontDrive;  //  Used to control the left front drive wheel
        private DcMotor rightFrontDrive;  //  Used to control the right front drive wheel
        private DcMotor leftBackDrive;  //  Used to control the left back drive wheel
        private DcMotor rightBackDrive;
        private DcMotor vertical1;  //  Used to control the left front drive wheel
        private DcMotor vertical2;  //  Used to control the right front drive wheel
        private DcMotor horizontal1;  //  Used to control the left back drive wheel
        private DcMotor horizontal2;//  Used to control the right back drive wheel

        private Servo claw1;
        private Servo claw2;
        private Servo wrist;
        private Servo arm;
        // Used to hold the data for a detected AprilTag

        @Override
        public void runOpMode() throws InterruptedException {


            boolean targetFound = false;    // Set to true when an AprilTag target is detected
            double drive = 0;        // Desired forward power/speed (-1 to +1)
            double strafe = 0;        // Desired strafe power/speed (-1 to +1)
            double turn = 0;        // Desired turning power/speed (-1 to +1)

            // Initialize the Apriltag Detection process

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must match the names assigned during the robot configuration.
            // step (using the FTC Robot Controller app on the phone).
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left front");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right front");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left rear");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right rear");
            horizontal1 = hardwareMap.get(DcMotor.class, "horizontal 1");
            horizontal2 = hardwareMap.get(DcMotor.class, "horizontal 2");
            vertical1 = hardwareMap.get(DcMotor.class, "vertical 1");
            vertical2 = hardwareMap.get(DcMotor.class, "vertical 2");
            claw1 = hardwareMap.get(Servo.class,"claw 1");
            arm = hardwareMap.get(Servo.class,"arm");
            claw2 = hardwareMap.get(Servo.class,"claw 2");
            wrist = hardwareMap.get(Servo.class,"wrist");
            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            vertical2.setDirection(DcMotor.Direction.REVERSE);
            horizontal2.setDirection(DcMotor.Direction.REVERSE);
            waitForStart();

            while (opModeIsActive()) {


                    double x = gamepad1.left_stick_x;
                    double y = gamepad1.left_stick_y;
                    double z = (gamepad1.right_stick_x *1.1);
                    double leftFrontPower    =  -x -y -z;
                    double rightFrontPower   =  -x +y +z;
                    double leftBackPower     =  -x -y -z;
                    double rightBackPower    =  -x +y +z;

                    // Normalize wheel powers to be less than 1.0
                    double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                    max = Math.max(max, Math.abs(leftBackPower));
                    max = Math.max(max, Math.abs(rightBackPower));


                    if (gamepad1.a) {
                    vertical1.setPower(1);
                        vertical2.setPower(0.4);
                        horizontal1.setPower(0);
                        horizontal2.setPower(0);
                    }
                    else if (gamepad1.b) {
                        horizontal1.setPower(0.5);
                        horizontal2.setPower(0.5);
                        vertical1.setPower(0);
                        vertical2.setPower(0);

                    }
                    else if (gamepad1.x) {
                        vertical1.setPower(-1);
                        vertical2.setPower(-0.4);
                        horizontal1.setPower(0);
                        horizontal2.setPower(0);
                    }

                    else if (gamepad1.y){
                        horizontal1.setPower(-0.5);
                        horizontal2.setPower(-0.5);
                        vertical1.setPower(0);
                        vertical2.setPower(0);
                    }
                    else if (gamepad1.left_bumper){
                        positions(0,180,90,-180);
                    }
                    else if (gamepad1.right_bumper){
                        positions(90,270,220,90);
                    }
                    else {
                        horizontal1.setPower(0);
                        horizontal2.setPower(0);
                        vertical1.setPower(0);
                        vertical2.setPower(0);
                    }



                    // Send powers to the wheels.
                    leftFrontDrive.setPower(leftFrontPower*0.4);
                    rightFrontDrive.setPower(rightFrontPower*0.4);
                    leftBackDrive.setPower(leftBackPower*0.4);
                    rightBackDrive.setPower(rightBackPower*0.4);





            }

        }
        private void positions (double servo1target,double servo2target, double armTarget, double wristtarget){
            claw1.setPosition(servo1target);
            claw2.setPosition(servo2target);
            arm.setPosition(armTarget);
            wrist.setPosition(wristtarget);
        }
    }


