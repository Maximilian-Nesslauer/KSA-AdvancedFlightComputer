using AdvancedFlightComputer.Guidance.Gfold;

internal static class TerminalBrakingTests
{
    internal static int Run()
    {
        int failures = 0;
        foreach (double step in new[] { 1.0 / 60, 0.05, 0.1 })
        foreach (double minimumTwr in new[] { 1.2, 3.0, 10.0 })
        foreach (double height in new[] { 100.0, 500.0 })
        foreach (double initialVelocity in new[] { 0.0, -20.0 })
        foreach (double groundBelowEstimate in new[] { 0.0, 3.0 })
            if (!Fly(height, initialVelocity, minimumTwr, step, groundBelowEstimate)) failures++;

        var rising = TerminalBraking.Evaluate(500, 15, 1.62, 4.8, 48, 1.0 / 60, 0.001, true);
        var stopped = TerminalBraking.Evaluate(100, 0, 1.62, 4.8, 48, 1.0 / 60, 0.001, true);
        var pulse = TerminalBraking.Evaluate(1, -2, 1.62, 16.2, 48, 1.0 / 60, 1, true);
        var settled = TerminalBraking.Evaluate(-0.08, -0.74, 1.62, 4.44, 48, 1.0 / 60, 0.001, false);
        if (rising.EngineOn || stopped.EngineOn || pulse.EngineOn || settled.EngineOn)
        {
            Console.WriteLine("FAIL: a rising craft, an early stop, an excessive minimum pulse, or a slow coast below the touchdown plane must coast.");
            failures++;
        }
        var belowPlane = TerminalBraking.Evaluate(-1.08, -2.02, 1.62, 4.44, 48, 1.0 / 60, 0.001, false);
        if (belowPlane.Acceleration != 4.44)
        {
            Console.WriteLine($"FAIL: below the touchdown plane a fast sink must brake at minimum thrust, got {belowPlane.Acceleration:F2}.");
            failures++;
        }
        Console.WriteLine($"Terminal braking checks: {failures} failure(s).");
        return failures == 0 ? 0 : 1;
    }

    // Integrate force and mass loss independently from the control law, including the engine's minimum pulse.
    // The controller sees a touchdown plane that lies groundBelowEstimate above the real ground.
    private static bool Fly(double initialHeight, double initialVelocity, double minimumTwr, double step, double groundBelowEstimate)
    {
        const double gravity = 1.62, exhaustVelocity = 4250, pulseTime = 0.001;
        double mass = 119000, height = initialHeight, velocity = initialVelocity, time = 0;
        double minimumThrust = mass * gravity * minimumTwr, maximumThrust = mass * 48;
        double firstIgnition = double.NaN, pulseRemaining = 0, thrust = 0;
        bool burning = false, climbed = false;
        int ignitions = 0;
        while (height > 0 && time < 180)
        {
            var command = TerminalBraking.Evaluate(height - groundBelowEstimate, velocity, gravity,
                minimumThrust / mass, maximumThrust / mass, step, pulseTime, burning);
            if (command.EngineOn)
            {
                if (!burning)
                {
                    ignitions++;
                    pulseRemaining = pulseTime;
                    if (double.IsNaN(firstIgnition)) firstIgnition = height;
                }
                thrust = command.Acceleration * mass;
            }
            else if (pulseRemaining <= 0) thrust = 0;
            burning = command.EngineOn;
            double acceleration = thrust / mass - gravity;
            height += velocity * step + 0.5 * acceleration * step * step;
            velocity += acceleration * step;
            mass -= thrust / exhaustVelocity * step;
            pulseRemaining = Math.Max(0, pulseRemaining - step);
            climbed |= velocity > 0.01;
            time += step;
        }
        // The controller decides once per step, so the contact sink can exceed its limit by one step of free fall.
        bool pass = height <= 0 && velocity >= -(TerminalBraking.ContactSinkLimit + gravity * step) && !climbed && firstIgnition < initialHeight * 0.5;
        Console.WriteLine($"{(pass ? "PASS" : "FAIL")}: height {initialHeight:F0}, initial sink {-initialVelocity:F0}, minimum TWR {minimumTwr:F1}, step {step:F3}, ground {groundBelowEstimate:F0} m below the estimate, ignition {firstIgnition:F2}, touchdown {-velocity:F2} m/s, starts {ignitions}, climb {climbed}");
        return pass;
    }
}
