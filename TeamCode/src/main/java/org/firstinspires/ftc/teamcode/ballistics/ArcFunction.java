package org.firstinspires.ftc.teamcode.ballistics;

import java.util.function.Function;

public interface ArcFunction extends Function<BallisticArc, Boolean> {
    @Override
    public Boolean apply(BallisticArc arc);
}