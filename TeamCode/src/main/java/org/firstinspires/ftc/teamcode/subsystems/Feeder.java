package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private final CRServo left = BaseOpMode.hardwareMap.crservo.get("Servo0");
    private final CRServo right = BaseOpMode.hardwareMap.crservo.get("Servo1");

    public Command feed() {
        return startEnd(() -> {
            left.setPower(1);
            right.setPower(-1);
        }, () -> {
            left.setPower(0);
            right.setPower(0);
        });
    }
}
