package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
DEVICE NAMES

WHEELS
right front wheel = "right_front_drive"
right back wheel = "right_back_drive"
left front wheel = "left_front_drive"
left back wheel = "left_back_drive"

ARM
arm rotation = "arm_motor_rotate"
arm extension = "arm_motor_extend"

INTAKE CLAW + WRIST
claw open & rotate = "claw_open_close"
claw horizontal rotation = "claw_rotate_h"
wrist = "wrist"
 */

@TeleOp(name="Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class OmniOpMode extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor armMotorRotate = null;
    private DcMotor armExtend = null;

    private Servo clawOpenClose;
    private Servo clawRotate;
    private Servo wrist;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");

        armMotorRotate = hardwareMap.get(DcMotor.class, "arm_motor_rotate");
        armExtend = hardwareMap.get(DcMotor.class, "arm_motor_extend");

        clawOpenClose = hardwareMap.get(Servo.class, "claw_open_close");
        clawRotate = hardwareMap.get(Servo.class, "claw_rotate_h");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial - lateral - yaw;
            double rightFrontPower = axial + lateral + yaw;
            double leftBackPower = axial + lateral - yaw;
            double rightBackPower = axial - lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 0.2 && gamepad1.a) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
                } else if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;

            } else if (max < 0.01) {
                leftFrontDrive.setPower(0);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                rightFrontDrive.setPower(0);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftBackDrive.setPower(0);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                rightBackDrive.setPower(0);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //ROTATE ARM//
            //Right bumper rotates arm up
            if (gamepad1.left_bumper) {
                armMotorRotate.setPower(-.8);
                telemetry.addData("Arm Direction", "rotate up");
            }

            //Left bumper rotates arm down
            else if (gamepad1.right_bumper) {
                armMotorRotate.setPower(.3);
                telemetry.addData("Arm Direction", "rotate down");

            }

            //Arm idle
            else {
                telemetry.addData("Arm Direction", "idle");
                armMotorRotate.setPower(0);
                armMotorRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //EXTEND ARM//
            double armMotorExtend;

            //Right/left trigger used for extending/retracting
            double extend = -gamepad1.right_trigger;
            double shorten = gamepad1.left_trigger;
            armMotorExtend = Range.clip(extend + shorten, -1.0, 1.0);

            armExtend.setPower(armMotorExtend);
            telemetry.addData("Arm Direction", "extend", extend);
            telemetry.addData("Arm Direction", "shorten", shorten);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);


            //OPEN INTAKE/CLAW
            clawOpenClose.scaleRange(-1, .2);

            //Close
            if (gamepad1.dpad_up) {
                clawOpenClose.setPosition(-1);
                telemetry.addData("Claw Servo Position","Open", clawOpenClose.getPosition());
            }

            //Open
            else if (gamepad1.dpad_down) {
                clawOpenClose.setPosition(.2);
                //telemetry.addData("Claw Servo Position","Close", clawOpenClose.getPosition());
            }

            //Idle
            else {
                telemetry.addData("Claw Servo Position", "Idle", clawOpenClose.getPosition());
            }
            //HORIZONTAL ROTATE INTAKE/CLAW
            clawRotate.scaleRange(-1, 1);

            //Move left
            if (gamepad1.dpad_left) {
                clawRotate.setPosition(0);
                //telemetry.addData("Claw Rotate Servo Position","Left", clawRotate.getPosition());
            }

            //Move right
            else if (gamepad1.dpad_right) {
                clawRotate.setPosition(-1);
                //telemetry.addData("Claw Rotate Servo Position","Right", clawRotate.getPosition());
            }

            //Move middle
            else if (gamepad1.right_stick_button){
                clawRotate.setPosition(.5);
                //telemetry.addData("Claw Rotate Servo Position","Middle", clawRotate.getPosition());
            }

            //Idle
            else {
                telemetry.addData("Claw Rotate Servo Position","Idle", clawRotate.getPosition());
            }


            //WRIST MOVEMENT
            wrist.scaleRange(-1, 1);

            //Move Wrist To Starting/Default Position
            if (gamepad1.a)  {
                wrist.setPosition(-.8);
                telemetry.addData("Wrist Position","Starting/Default Position", wrist.getPosition());
            }

            //Move Wrist Out/Up Position
            else if (gamepad1.y) {
                wrist.setPosition(.8);
                telemetry.addData("Wrist Position","Outward/Up", wrist.getPosition());
            }

            //Idle
            else {
                telemetry.addData("Wrist","Idle", wrist.getPosition());
            }
            telemetry.update();
        }
    }
}

