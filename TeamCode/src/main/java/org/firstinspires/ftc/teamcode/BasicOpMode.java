package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class BasicOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;

    private DcMotor armMotorRotate = null;
    private DcMotor armExtend = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDriveFront  = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        leftDriveBack  = hardwareMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_drive_back");
        armMotorRotate = hardwareMap.get(DcMotor.class, "arm_motor_rotate");
        armExtend = hardwareMap.get(DcMotor.class, "arm_motor_extend");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    //Motor stuff/movement
    public void loop() {

        //WHEELS//
        double leftPowerFront;
        double rightPowerFront;
        double leftPowerBack;
        double rightPowerBack;

        // - Only left stick is used.
        // Neg = reverse  positive = foreword
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        leftPowerFront = Range.clip(drive + turn, -0.7, 1.0) ;
        rightPowerFront = Range.clip(drive - turn, -0.7, 1.0) ;
        leftPowerBack = Range.clip(drive + turn, -0.7, 1.0) ;
        rightPowerBack = Range.clip(drive - turn, -0.7, 1.0) ;

        // Send calculated power to wheels
        leftDriveFront.setPower(leftPowerFront);
        leftDriveBack.setPower(leftPowerBack);
        rightDriveFront.setPower(rightPowerFront);
        rightDriveBack.setPower(rightPowerBack);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back (%.2f)", leftPowerFront, leftPowerBack, rightPowerFront, rightPowerBack);

        //ROTATE ARM//
        double armPower = 0.3;

        //Right bumper rotates arm down
        if(gamepad1.left_bumper){
            armMotorRotate.setPower(-armPower);
            telemetry.addData("Arm Direction", "rotate down");
        }

        //Right bumper rotates arm up
        else if(gamepad1.right_bumper){
            armMotorRotate.setPower(armPower);
            telemetry.addData("Arm Direction", "rotate up");
        }

        //Arm idle
        else {
            telemetry.addData("Arm Direction", "idle");
            armMotorRotate.setPower(0);
        }

        //EXTEND ARM//
        double armMotorExtend;

        //Right joystick used for extending/shortening
        double extend = -gamepad1.right_stick_y;
        double shorten  =  gamepad1.right_stick_x;
        armMotorExtend = Range.clip(extend + shorten, -1.0, 1.0);

        armExtend.setPower(armMotorExtend);
        telemetry.addData("Arm Direction", "extend",extend);
        telemetry.addData("Arm Direction", "shorten",shorten);
    }

    @Override
    public void stop() {
    }

}
