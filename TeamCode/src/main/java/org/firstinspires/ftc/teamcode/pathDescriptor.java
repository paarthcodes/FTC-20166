package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class pathDescriptor
{
    public static Path pathToFollow;
    public static double finalHeading;

    public pathDescriptor(Path pathToFollow, double finalHeading)
    {
        this.pathToFollow = pathToFollow;
        this.finalHeading = finalHeading;
    }
}
