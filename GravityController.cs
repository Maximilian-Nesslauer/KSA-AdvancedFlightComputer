using Brutal.GlfwApi;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using Brutal.Logging;
using HarmonyLib;
using KSA;
using RenderCore;
using RenderCore.Input;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using System.Text;
using System.Text.Json.Nodes;
using static Brutal.Strings.Utf8;

namespace NovaTec.GravityTurnMod
{
    public class GravityController
    {
        /*
        public double InitialPitch { get; set; } = 25.0;
        public double InitialSpeed { get; set; } = 100.0;
        public int TimeToApoapsisStart { get; set; } = 70;
        public int TimeToApoapsisEnd { get; set; } = 70;
        public int TimeToApoapsisTarget { get; set; } = 70;
        public double TargetAltitude { get; set; } = 280.0;
        */
        /* works for RSS size*/
        public double InitialPitch { get; set; } = 9;
        public double InitialSpeed { get; set; } = 95;
        public int TimeToApoapsisStart { get; set; } = 65;
        public int TimeToApoapsisEnd { get; set; } = 65;
        public int TimeToApoapsisTarget { get; set; } = 70;
        public double TargetAltitude { get; set; } = 280;
        public double MinThrottle{ get; set; } = 0.2;
        public double TargetInclination { get; set; } = 0;
        public double LaunchAzimuth{ get; set; } = 0.0;
        public bool UseWarp { get; set; } = true;
        public bool AutoStage{ get; set; } = true;

        public double DeltaVUsed { get { return _DeltaVUsed; } }
        public bool PitchesUp{ get; set; } = false;
        public enum PhaseEnum
        {
            Landed, Initial, Pitch, Stage, Hold, Coast, Circularize, Cleanup, Idle
        }

        public PhaseEnum Phase = PhaseEnum.Landed;

        private Vehicle? ControlledVehicle = null;

        private double LaunchAltitude = 0.0;

        public double LastTransitionTime = Universe.GetElapsedSeconds();

        private double DeltaVAtLast = 0.0;
        private double _DeltaVUsed = 0.0;

        public GravityController(Vehicle vehicle) 
        {
            Phase = PhaseEnum.Landed;
            ControlledVehicle = vehicle;
        }
        public void SetVehicle(Vehicle vehicle)
        {
            ControlledVehicle = vehicle;
        }

        public void Launch(Vehicle vehicle)
        {
            if (vehicle == null)
                return;

            LaunchAltitude = GetAltitude();
            LaunchAltitude = ControlledVehicle.GetBarometricAltitude();
            LaunchAzimuth = CalculateLaunchAzimuth(TargetInclination);

            TimeToApoapsisTarget = TimeToApoapsisStart;
            DeltaVAtLast = vehicle.NavBallData.DeltaV;
            _DeltaVUsed = 0;

            Console.WriteLine("Launch vehicle");
            vehicle.SetStabilization(true);
            PatchRcsPriority.Active = true;

            FlightControlOverride.Active = true;
            FlightControlOverride.RCSMode = FlightComputerRCSMode.Enabled;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Up;
            FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EnuBody;

            Console.WriteLine("Active Sequence: " + vehicle.Parts.SequenceList.ActiveSequence);
            if (vehicle.Parts.SequenceList.ActiveSequence <= 0)
            {
                Console.WriteLine("Next Sequence: " + vehicle.Parts.SequenceList.ActiveSequence);
                NextStequence();
            }

            // Ignite engines
            ThrottleOverride.Active = true;
            SetEngineThrottle(0.5);
            IgniteEngines();

            Phase = PhaseEnum.Initial;
            LastTransitionTime = Universe.GetElapsedSeconds();
        }

        public void Run()
        {
            SetVehicle(Program.ControlledVehicle);
            Vehicle? vehicle = ControlledVehicle;
            if (vehicle == null)
                return;

            switch (Phase)
            {
                case PhaseEnum.Landed:
                    RunPhaseLanded(vehicle); break;
                case PhaseEnum.Initial:
                    RunPhaseInitial(vehicle); break;
                case PhaseEnum.Pitch:
                    RunPhasePitch(vehicle); break;
                case PhaseEnum.Stage:
                    RunPhaseStage(vehicle); break;
                case PhaseEnum.Hold:
                    RunPhaseHold(vehicle); break;
                case PhaseEnum.Coast:
                    RunPhaseCoast(vehicle); break;
                case PhaseEnum.Circularize:
                    RunPhaseCircularize(vehicle); break;
                case PhaseEnum.Cleanup:
                    RunPhaseCleanup(vehicle); break;
            }

            // hack to simulate lower TWR engines
            if (GetAtmosphereHeight() > 10 && GetAltitude() < GetAtmosphereHeight()/3 && (Phase != PhaseEnum.Landed && Phase != PhaseEnum.Idle))
            {
                if (vehicle.NavBallData.ThrustWeightRatio > this.DeltaVUsed / 600 + 1.75 || vehicle.NavBallData.ThrustWeightRatio > 3)
                {
                    ThrottleDown();
                }
                else if (vehicle.NavBallData.ThrustWeightRatio < this.DeltaVUsed / 600 + 1.65 && GetApoapsisTime() < TimeToApoapsisStart)
                {
                    ThrottleUp();
                }
            }
            
            vehicle.UpdatePerFrameData();

        }

        private void RunPhaseLanded(Vehicle vehicle)
        {
            LaunchAltitude = GetAltitude();
            // do nothing, maybe collect stats later
        }

        //
        // during this phase the vehicle launches directly upwards until it reaches the pitch altitude
        private void RunPhaseInitial(Vehicle vehicle)
        {

            if (UseWarp && vehicle.GetBarometricAltitude() > 100 && Universe.SimulationSpeed < 2 )
                Universe.SetSimulationSpeed(2.0, false);

            if (GetSpeed() > InitialSpeed)
            {
                StartPhasePitch(vehicle);
            }
        }

        public void StartPhasePitch(Vehicle vehicle)   
        {
            Console.WriteLine("PHASE: Pitch over");

            Phase = PhaseEnum.Pitch;
            LastTransitionTime = Universe.GetElapsedSeconds();
            if (UseWarp)
                Universe.SetSimulationSpeed(1.0, false);
            vehicle.SetStabilization(true);

            // create custom target for pitch
            double azimuth = LaunchAzimuth;

            double3 target = new double3(0, DegToRad(90 - InitialPitch), DegToRad(azimuth));
            FlightControlOverride.Active = true;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
            FlightControlOverride.RollMode = FlightComputerRollMode.Up;
            FlightControlOverride.CustomAttitudeTarget = target;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
            FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EnuBody;

        }

        private void RunPhasePitch(Vehicle vehicle)
        {
            //CalculateStats();
            double pitch = vehicle.BodyRates.X;

            double diff = Universe.GetElapsedSeconds() - LastTransitionTime;
            if (UseWarp && diff > 3 && Universe.SimulationSpeed < 1.1)
                Universe.SetSimulationSpeed(2.0, false);

            if (diff > InitialPitch * 0.8 /*&& Math.Abs(vehicle.BodyRates.X) < 0.005 && Math.Abs(vehicle.BodyRates.Y) < 0.005*/)
            {
                StartPhaseHold(vehicle);
            }
        }
        public void StartPhaseHold(Vehicle vehicle)
        {
            Console.WriteLine("PHASE: Hold");
            if (UseWarp)
                Universe.SetSimulationSpeed(4.0, false);

            FlightControlOverride.Active = true;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
            FlightControlOverride.RollMode = FlightComputerRollMode.Up;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Forward;
            FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EnuBody;

            LastTransitionTime = Universe.GetElapsedSeconds();
            Phase = PhaseEnum.Hold;
        }

        double lastTimeToApoapsis = 0;
        double fwdPitch = 0;
        double iniPitch = 0;
        bool didReachTargetApoapsisTime = false;

        private void RunPhaseHold(Vehicle vehicle)
        {
            CalculateStats();

            // needs staging?
            if (AutoStage && vehicle.Parts.SequenceList.ActiveSequence > 0 && !GetSequenceHasFuel())
            {
                StartPhaseStage(vehicle);
                return;
            }

            // roll is at nearly 0?
            if (Math.Abs(GetRoll()) < 6 && vehicle.FlightComputer.ActiveControlSystem.X == AttitudeControlSystem.Rcs)
                PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Tvc;
            else if (Math.Abs(GetRoll()) > 6 && vehicle.FlightComputer.ActiveControlSystem.X != AttitudeControlSystem.Rcs)
                PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Rcs;

            double phaseDuration = Universe.GetElapsedSeconds() - LastTransitionTime;
            double diff = GetApoapsisTime() - lastTimeToApoapsis;

            // now check time to apoapsis
            if (GetApoapsisTime() > TimeToApoapsisTarget && vehicle.GetManualThrottle() > MinThrottle && phaseDuration > 5)
            {
                didReachTargetApoapsisTime = true;
                ThrottleDown();
            }

            // adjust target AP time based on current AP time
            if (GetApoapsisTime() > TimeToApoapsisTarget + 5 && diff > 0)
                TimeToApoapsisTarget = (int)GetApoapsisTime();
            // throttle up if AP time is below target
            if (GetApoapsisTime() < TimeToApoapsisTarget)
                ThrottleUp();


            // do pitch up or down if needed. If that is not enough, it's an indicator of wrong startup values.
            // In general the need to pitch up is a sign of a weak 2nd stage.
            //if (vehicle.Parts.SequenceList.ActiveSequence > 1 && didReachTargetApoapsisTime)
            {
                // if tta is decreasing with full throttle then pitch up
                if (GetApoapsisTime() < TimeToApoapsisStart - 1 && vehicle.GetManualThrottle() >= 1 && diff < 0)
                {
                    double pitch = (double)Program.AttitudePitch.Current;
                    if (iniPitch.IsNearlyZero())
                        iniPitch = pitch;
                    else
                        pitch = iniPitch;
                    
                    if ((TimeToApoapsisStart - GetApoapsisTime()) > 0)
                        fwdPitch = pitch - (TimeToApoapsisStart - GetApoapsisTime()) / 2.0;
                    else
                        fwdPitch = pitch;

                    double3 target = new double3(0, DegToRad(-fwdPitch), vehicle.Orbit.Inclination);
                    FlightControlOverride.CustomAttitudeTarget = target;
                    if (FlightControlOverride.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Custom)
                    {
                        FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
                    }
                    PitchesUp = true;
                }
                // reduce pitch if it's cathing up
                else if (GetApoapsisTime() < TimeToApoapsisStart - 1 && vehicle.GetManualThrottle() >= 1 && diff > 0)
                {
                    double pitch = (double)Program.AttitudePitch.Current;
                    if (iniPitch.IsNearlyZero())
                        iniPitch = pitch;
                    else
                        pitch = iniPitch;

                    if ((TimeToApoapsisStart - GetApoapsisTime()) > 0)
                        fwdPitch = pitch - (TimeToApoapsisStart - GetApoapsisTime()) / 5;
                    else
                        fwdPitch = pitch;

                    double3 target = new double3(0, DegToRad(-fwdPitch), vehicle.Orbit.Inclination);
                    FlightControlOverride.CustomAttitudeTarget = target;
                    if (FlightControlOverride.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Custom)
                    {
                        FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
                    }
                    PitchesUp = true;
                }
                // stop pitching up if we are above half of the target AP time and TTA is increasing
                else if (GetApoapsisTime() > (TimeToApoapsisTarget + TimeToApoapsisTarget) / 2 && diff > 0)
                {
                    iniPitch = 0;
                    fwdPitch = 0;
                    if (FlightControlOverride.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Forward)
                    {
                        FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Forward;
                    }

                    PitchesUp = false;
                }
            }

            // reduce throttle if AP is close to target altitude - better to use rcs only, but not for now...
            if (GetApoapsisAltitude() / 1000 > TargetAltitude * 0.90 && didReachTargetApoapsisTime && GetApoapsisTime() > TimeToApoapsisTarget + TimeToApoapsisStart*2)
            {
                ThrottleOverride.Throttle = 0.05f;
            }

            // change warp, depending on how close we are to target AP time and target AP altitude
            if (UseWarp)
            {
                // slow warp if deltaV is low 
                if (vehicle.NavBallData.DeltaV < 450)
                {
                    if (Universe.SimulationSpeed > 9)
                        Console.WriteLine("Slowdown warp before staging");
                    var warp = Math.Clamp(vehicle.NavBallData.DeltaV / 50.0, 1.0, 9.0);
                    //Console.WriteLine("Slowdown warp {0}, {1}, {2}", warp, vehicle.NavBallData.DeltaV, vehicle.NavBallData.DeltaV / 400.0);
                    Universe.SetSimulationSpeed(warp, false);
                }
                // slow warp just after staging
                else if ((phaseDuration <= 9 && GetCurrentSequence()?.Number > 1))
                {
                    if (Universe.SimulationSpeed > 9)
                        Console.WriteLine("Rampup warp after staging");
                    Universe.SetSimulationSpeed(Math.Clamp(phaseDuration, 1.0, 9.0), false);
                }
                // reduce throttle if AP is close to target altitude and set sim speed to 1
                else if (GetApoapsisAltitude() / 1000 > TargetAltitude * 0.90 && didReachTargetApoapsisTime)
                {
                    if (Universe.SimulationSpeed > 9)
                        Console.WriteLine("Slowdown warp, AP close to target");
                    double warp = (1.0 - (GetApoapsisAltitude() / 1000 / TargetAltitude)) * 90.0;
                    Universe.SetSimulationSpeed(Math.Clamp(warp, 1.0, 10.0), false);
                }
                // increase sim speed if AP is above lower atmosphere
                else if (didReachTargetApoapsisTime && Universe.SimulationSpeed < 10 && !PitchesUp)
                {
                    Console.WriteLine("Speedup to warp 10");
                    Universe.SetSimulationSpeed(10.0, false);
                }
                if (PitchesUp)
                {
                    Universe.SetSimulationSpeed(2.0, false);
                }
            }
            // transition to coast if AP target altitude is reached.
            if (GetApoapsisAltitude() / 1000 > TargetAltitude)
            {
                ShutdownEngines();
                StartPhaseCoast(vehicle);
            }

            lastTimeToApoapsis = GetApoapsisTime();

        }

        private void StartPhaseStage(Vehicle vehicle)
        {
            Console.WriteLine("PHASE: Stage");
            if (UseWarp)
                Universe.SetSimulationSpeed(1.0, false);
            LastTransitionTime = Universe.GetElapsedSeconds();
            Phase = PhaseEnum.Stage;
        }
        private void RunPhaseStage(Vehicle vehicle)
        {
            double diff = Universe.GetElapsedSeconds() - LastTransitionTime;

            SetEngineThrottle(1.0);
            // Wait for staging to settle wobble...
            if (AutoStage && vehicle.Parts.SequenceList.ActiveSequence > 0 && !GetSequenceHasFuel() && diff > 0.3)
            {
                LastTransitionTime = Universe.GetElapsedSeconds();
                Console.WriteLine("Trigger decoupler, then light engine");
                NextStequence();
                return;
            }
            // Wait for ignition to get clear of previous stage
            else if (diff > 0.8)
            {
                // no fuel in this stage, then it's probably a decoupler, so skip to next stage
                if (AutoStage && !GetSequenceHasFuel())
                {
                    Console.WriteLine("Trigger decoupler, then light engine");
                    NextStequence();
                }

                if ((int)GetApoapsisTime() > TimeToApoapsisStart)
                    TimeToApoapsisTarget = (int)GetApoapsisTime();
                StartPhaseHold(vehicle); // back to hold mode
            }

        }
        private void StartPhaseCoast(Vehicle vehicle)
        {
            Console.WriteLine("PHASE: Coast");
            ShutdownEngines();

            LastTransitionTime = Universe.GetElapsedSeconds();
            Phase = PhaseEnum.Coast;

            FlightControlOverride.Active = true;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
            FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EclBody;
            FlightControlOverride.RollMode = FlightComputerRollMode.Up;

        }
        private void RunPhaseCoast(Vehicle vehicle)
        {
            if (GetApoapsisAltitude() / 1000 < TargetAltitude - 5)
            {
                SetEngineThrottle(0.1);
                IgniteEngines();
            }
            else if (GetApoapsisAltitude() / 1000 > TargetAltitude + 5)
            {
                SetEngineThrottle(0.1);
                ShutdownEngines();
            }

            if (UseWarp && Universe.SimulationSpeed < 10.5)
            {
                Universe.SetSimulationSpeed(Math.Clamp(Universe.GetElapsedSeconds() - LastTransitionTime, 1.0, 10.0), false);
            }

            if (GetAltitude() > GetAtmosphereHeight())
            {
                StartPhaseCircularize(vehicle);
            }

        }
        public void StartPhaseCircularize(Vehicle vehicle)
        {
            Console.WriteLine("PHASE: Circularize");
            Phase = PhaseEnum.Circularize;
            LastTransitionTime = Universe.GetElapsedSeconds();

            double currentAp = vehicle.Apoapsis;
            double currentPe = vehicle.Periapsis;
            double targetR = currentAp;

            FlightComputer fc = vehicle.FlightComputer;
            
            ThrottleOverride.Active = true;

            // create burn to circularize
            double3 dV = OrbitalTransfers.DvCciToCircularize(vehicle.Orbit, vehicle.NextApoapsisTime);
            double ignitionOffset = 0;
            // if we need to stage, then we have to adapt the burn time and ignition time
            if (vehicle.Parts.PerformanceSequences.FindActiveSequenceDeltaV() < dV.Length())
            {
                Console.WriteLine("need to stage => make burn 30 seconds earlier");
                ignitionOffset = 30;
            }

            if (dV.Length() > 500)
                SetEngineThrottle(dV.Length() / 500.0);
            else
                SetEngineThrottle(1.0);

            Console.WriteLine("Circularization dV X:" + dV.X + ", Y: " + dV.Y + ", Z: " + dV.Z + ", r2: " + dV.Length());
            OrbitPointCce point = new OrbitPointCce(vehicle.Orbit.GetApoapsisPositionOrb(), vehicle.TimeSincePeriapsis, vehicle.NextApoapsisTime - Universe.GetElapsedTime(), TrueAnomaly.NaN);
            PatchedConic patch = new PatchedConic(vehicle.NextApoapsisTime, vehicle.NextApoapsisTime, PatchTransition.Burn, PatchTransition.Burn, vehicle.Orbit, KeyHash.Make(new ReadOnlySpan<char>("Circularize".ToArray())));
            Burn burn = Burn.Create(point,
                vehicle.NextApoapsisTime.Seconds()-ignitionOffset,
                new double3(dV.Length()*1.0, 0, 0),
                patch,
                vehicle);

            fc.AddBurn(burn);
            Console.WriteLine("  Duration: {0} s", fc.Burn?.BurnDuration);


            PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.None;
            FlightControlOverride.RCSMode = FlightComputerRCSMode.Disabled;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
            FlightControlOverride.Active = true;
            ThrottleOverride.Active = false;


            if (UseWarp)
            {
                Universe.SetSimulationSpeed(10.0, false);
            }

        }

        private void RunPhaseCircularize(Vehicle vehicle)
        {
            FlightComputer fc = vehicle.FlightComputer;

            double secondsToIgnition = fc.Burn != null ? (fc.Burn.IgnitionTime - Universe.GetElapsedTime()).Seconds() : 0;

            if (UseWarp && fc.Burn != null)
            {
                const double warpSpeed = 120;
                // speedup warp if ignition is far away
                if (secondsToIgnition >= warpSpeed * 2 && !Universe.IsAutoWarpActive && Universe.GetSimulationSpeed() < warpSpeed)
                {
                    double warp = (Universe.GetElapsedSeconds() - LastTransitionTime) * 1.5 + 10.0;
                    if (Universe.GetSimulationSpeed() <= 10.0)
                        Console.WriteLine("Warp to burn ignition in {0} s with warp {1,3:N}", secondsToIgnition, warp);
                    Universe.SetSimulationSpeed(Math.Clamp(warp, 10.0, warpSpeed), false);
                }
                // slow down warp if closer to ignition
                else if (secondsToIgnition < warpSpeed * 2 && secondsToIgnition >= 60 && !Universe.IsAutoWarpActive && Universe.GetSimulationSpeed() >= 4)
                {
                    if (Universe.GetSimulationSpeed() == warpSpeed)
                        Console.WriteLine("Slowdown warp to burn ignition in {0,3:N} s", secondsToIgnition);
                    double warp = Universe.GetSimulationSpeed() / 1.011;
                    Universe.SetSimulationSpeed(Math.Clamp(warp, 4.1, warpSpeed), false);
                }
                else if (secondsToIgnition >= 3 && !Universe.IsAutoWarpActive && Universe.GetSimulationSpeed() < 1.1)
                {
                    if (fc.Burn.BurnDuration > 20)
                        Universe.SetSimulationSpeed(4.0, false);
                    else if (fc.Burn.BurnDuration > 10)
                        Universe.SetSimulationSpeed(2.0, false);
                    else
                        Universe.SetSimulationSpeed(1.0, false);
                }
            }

            // Burn is about to start, so stop auto warp and set speed to 1x
            if (secondsToIgnition < 60 && (Universe.IsAutoWarpActive || Universe.GetSimulationSpeed() > 4))
            {
                FlightControlOverride.Active = true;
                FlightControlOverride.RCSMode = FlightComputerRCSMode.Enabled;
                FlightControlOverride.BurnMode = FlightComputerBurnMode.Auto;
                PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Tvc;
                FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
                FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EclBody;
                if (UseWarp)
                {
                    if (Universe.IsAutoWarpActive)
                        Universe.AutoWarpStop(true);
                    Universe.SetSimulationSpeed(4.0, false);
                }
            }
            if (secondsToIgnition < 2 && (Universe.IsAutoWarpActive || Universe.GetSimulationSpeed() > 1))
            {
                Console.WriteLine("Burn close to ignition");
                Console.WriteLine("   Burn Duration: {0} burns: {1}", fc.Burn?.BurnDuration, fc.BurnPlan.BurnCount);
                if (UseWarp)
                {
                    Universe.SetSimulationSpeed(1.0, false);
                    Universe.AutoWarpStop(true);
                }
                FlightControlOverride.Active = true;
                FlightControlOverride.BurnMode = FlightComputerBurnMode.Auto;
                FlightControlOverride.RCSMode = FlightComputerRCSMode.Enabled;
            }
            // burn completed?
            else if (fc.Burn == null || fc.Burn.BurnDuration <= 0.01f || fc.BurnPlan.BurnCount == 0)
            {
                Console.WriteLine("\nBurn complete? Burns: {0}", fc.BurnPlan.BurnCount);
                Console.WriteLine("  Duration: {0}", fc.Burn?.BurnDuration);
                FlightControlOverride.Active = true;
                FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
                FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
                FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EclBody;
                FlightControlOverride.AttitudeMode = FlightComputerAttitudeMode.Auto;
                FlightControlOverride.RCSMode = FlightComputerRCSMode.Disabled;

                Phase = PhaseEnum.Cleanup;
                LastTransitionTime = Universe.GetElapsedSeconds();
            }
            else if (secondsToIgnition < 0)
            {
                Console.Write("   Burn Duration left: {0} engine: {1}  \r", fc.Burn?.BurnDuration, vehicle.IsAnyEngineActive());
                // needs staging?
                if (vehicle.Parts.SequenceList.ActiveSequence > 0 && !GetSequenceHasFuel() && AutoStage)
                {
                    Console.WriteLine("\nNo fuel, stage!");
                    StartPhaseStage(vehicle);
                }
            }
        }

        public void RunPhaseCleanup(Vehicle vehicle)
        {
            ThrottleOverride.Active = true;
            SetEngineThrottle(1.0);
            if (Universe.GetElapsedSeconds() - LastTransitionTime > 1)
            {
                SetEngineThrottle(1.0);
                Console.WriteLine("Gravity turn cleanup done.");

                FlightComputer fc = vehicle.FlightComputer;
                if (fc.BurnPlan.BurnCount > 0)
                {
                    fc.BurnMode = FlightComputerBurnMode.Manual;
                    Console.WriteLine("Burn complete, deleting burn [{0}], active {1}", fc.BurnPlan.BurnCount - 1, fc.BurnPlan.HasActiveBurns);
                    if (Program.ActiveBurn != null)
                    {
                        fc.RemoveBurn(Program.ActiveBurn);
                        fc.Burn = null;
                    }
                    Console.WriteLine("  burns left: {0}", fc.BurnPlan.BurnCount);
                    fc.BurnPlan.Clear();
                    Console.WriteLine("Burn complete, burn count {0}, active {1}", fc.BurnPlan.BurnCount, fc.BurnPlan.HasActiveBurns);
                }

                // Cleanup logic for the cleanup phase
                vehicle.FlightComputer.RCSMode = FlightComputerRCSMode.Disabled;
                PatchRcsPriority.Active = false;
                FlightControlOverride.Active = false;
                ThrottleOverride.Active = false;

                Phase = PhaseEnum.Idle;
            }

        }

        public void RunWorker()
        {
            ControlledVehicle?.PrepareWorker(Universe.GetNextSimStep());

        }

        public void ThrottleUp()
        {
            var vehicle = ControlledVehicle;
            if (vehicle is null) return;
            SetEngineThrottle(vehicle.GetManualThrottle() + 0.005);
        }
        public void ThrottleDown()
        {
            var vehicle = ControlledVehicle;
            if (vehicle is null) return;
            SetEngineThrottle(vehicle.GetManualThrottle() - 0.005);
        }
        public void SetEngineThrottle(double throttleValue)
        {
            float clampedValue = (float)Math.Clamp(throttleValue, 0.0, 1.0);
            ControlledVehicle?.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            ThrottleOverride.Throttle = clampedValue;
        }

        public void IgniteEngines()
        {
            ThrottleOverride.Active = true;
            ThrottleOverride.EngineOn = true;
            //ControlledVehicle?.SetEnum(VehicleEngine.MainIgnite);
        }
        public void ShutdownEngines()
        {
            ThrottleOverride.EngineOn = false;
            //ControlledVehicle?.SetEnum(VehicleEngine.MainShutdown);
            ThrottleOverride.Active = true;
        }

        //-------------------------------------------------------------------------

        public void CalculateStats()
        {
            // TODO: fix ContainsEngine
            if (ControlledVehicle == null || GetCurrentSequence() == null || /*!GetCurrentSequence().ContainsEngine ||*/ ControlledVehicle.NavBallData.DeltaV <= 0)
                return;

            if (DeltaVAtLast < ControlledVehicle.NavBallData.DeltaV)
                DeltaVAtLast = ControlledVehicle.NavBallData.DeltaV;


            _DeltaVUsed += DeltaVAtLast - ControlledVehicle.NavBallData.DeltaV;
            DeltaVAtLast = ControlledVehicle.NavBallData.DeltaV;
        }

        static public String CoordToString(double3 angles)
        {
            double3 degrees = new double3
            {
                X = RadToDeg(angles.X),
                Y = RadToDeg(angles.Y),
                Z = RadToDeg(angles.Z)
            };
            return String.Format("X: {0,6:N}, >: {1,6:N}, Z: {2,6:N}", degrees.X, degrees.Y, degrees.Z);
        }
        static public String AnglesToString(double3 angles)
        {
            double3 degrees = new double3
            {
                X = RadToDeg(angles.X),
                Y = RadToDeg(angles.Y),
                Z = RadToDeg(angles.Z)
            };
            return String.Format("X: {0,6:N}, >: {1,6:N}, Z: {2,6:N}", degrees.X, degrees.Y, degrees.Z);
        }

        public double CalculateEquatoriaRotationSpeed()
        {
            Vehicle? vehicle = ControlledVehicle;
            if (vehicle == null || vehicle.Orbit == null || vehicle.Orbit.Parent == null)
                return 0;
            IParentBody parent = vehicle.Orbit.Parent;
            if (parent.GetAtmosphereReference() == null)
                return 0;
            double lat = RadToDeg(vehicle.Orbit.Inclination);
            double angularVelocity = parent.GetAngularVelocity();
            double equatorialSpeed = Math.Cos(DegToRad(lat)) * angularVelocity * parent.MeanRadius;
            Console.WriteLine("Equatorial speed: " + equatorialSpeed);
            return equatorialSpeed;
        }
        public double CalculateLaunchAzimuth(double inclination)
        {
            Vehicle? vehicle = ControlledVehicle;
            if (vehicle == null || vehicle.Orbit == null || vehicle.Orbit.Parent == null)
                return 0;
            IParentBody parent = vehicle.Orbit.Parent;
            double latitude = RadToDeg(vehicle.Orbit.Inclination);

            if (inclination == double.NaN || inclination == 0.0)
            {
                DefaultCategory.Log.Info(String.Format("Launch east: {0}", latitude));
                return 0.0;
            }

            double inertial = Math.Asin(Math.Cos(DegToRad(inclination)) / Math.Cos(DegToRad(latitude)));
            if (inertial == double.NaN)
            {
                DefaultCategory.Log.Info(String.Format("Launch east: {0}", latitude));
                return 0.0;
            }
            inertial = RadToDeg(inertial);

            Console.WriteLine(String.Format("Launch into inclination: {0}", inclination));
            Console.WriteLine("   latitude: " + latitude);
            Console.WriteLine("   inertial: " + inertial);
            Console.WriteLine("   planet radius: " + vehicle.Parent.MeanRadius);
            var vOrbit = GetOrbitalSpeed(vehicle.Parent.MeanRadius + TargetAltitude * 1000);
            Console.WriteLine("   orbital speed: " + vOrbit);
            var vAngular = vehicle.Parent.GetAngularVelocity();
            double vEquator = vAngular * parent.MeanRadius;
            double vLatitude= Math.Cos(DegToRad(latitude)) * vEquator;
            Console.WriteLine("   angular velocity: " + vAngular);
            Console.WriteLine("   equatorial speed: " + vEquator);
            Console.WriteLine("   latitude speed: " + vLatitude);

            var vXrot = vOrbit * Math.Sin(DegToRad(inertial)) - vEquator * Math.Cos(DegToRad(latitude));
            Console.WriteLine("   vXrot: " + vXrot);
            var vYrot = vOrbit * Math.Cos(DegToRad(inertial));
            Console.WriteLine("   vYrot: " + vYrot);
            var azimuth = RadToDeg(Math.Atan(vXrot / vYrot))-90;
            Console.WriteLine("Launch azimuth: " + azimuth);

            return azimuth;
        }

        //-------------------------------------------------------------------------

        public double GetOrbitalSpeed(double radiusMeters)
        {
            if (ControlledVehicle == null)
                return 0.0;

            // assume circular orbit
            double semiMajorAxis = radiusMeters;
            return Math.Sqrt(ControlledVehicle.Orbit.Mu * (2.0 / radiusMeters - 1.0 / semiMajorAxis));
        }

        public Sequence? GetCurrentSequence()
        {
            Vehicle? vehicle = ControlledVehicle;
            Sequence? sequence = null;
            if (vehicle == null || vehicle.Parts == null || vehicle.Parts.SequenceList == null)
                return null;

            if (vehicle.Parts.SequenceList.ActiveSequence > 0
             && vehicle.Parts.SequenceList.Count > vehicle.Parts.SequenceList.ActiveSequence)
            {
                sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.ActiveSequence - 1];
            }
            else if (vehicle.Parts.SequenceList.Count > 0)
            {
                sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.Count - 1];
            }
            return sequence;
        }

        public Sequence? NextStequence()
        {
            Vehicle? vehicle = ControlledVehicle;
            Sequence? sequence = null;
            if (vehicle == null) 
                return null;

            if (vehicle.Parts.SequenceList.ActiveSequence <= 0)
            {
                vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            }
            else if (vehicle.Parts.SequenceList.ActiveSequence > 0)
            {
                int prevSequence = vehicle.Parts.SequenceList.ActiveSequence;
                vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
                if (GetSequenceHasFuel())
                   vehicle.Parts.SequenceList.SetActiveSequence(prevSequence);
            }

            vehicle.UpdateAfterPartTreeModification();
            vehicle.Parts.SequenceList.RemoveSpentSequences();

            if (vehicle.Parts.SequenceList.ActiveSequence > 0)
            {
                Console.WriteLine("Activated sequence: " + vehicle.Parts.SequenceList.ActiveSequence);
                Console.WriteLine("  Engines: " + GetEngineControllers().Count);

                if (GetEngineControllers().Count > 0)
                {
                    IgniteEngines();
                }
            }

            return sequence;
        }

        public float GetFuelInSequence()
        {
            Vehicle? vehicle = Program.ControlledVehicle;
            if (vehicle == null)
                return 0.0f;
            Sequence sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.ActiveSequence - 1];
            foreach (Part p in sequence.Parts)
            {
                Span<Tank> tanks = p.SubtreeModules.Get<Tank>();
                if (tanks.Length > 0)
                {
                    Tank tank = tanks[0];
                    return tank.Moles[0].ContainerVolume;
                }

            }
            return 0;

        }
        public ArrayList GetEngineControllers()
        {
            ArrayList engines = new ArrayList();
            Vehicle? vehicle = Program.ControlledVehicle;
            if (vehicle == null || vehicle.Parts.SequenceList.ActiveSequence < 1) 
                return engines;
            Sequence? sequence = GetCurrentSequence();
            //Sequence sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.ActiveSequence - 1];
            if (sequence == null) return engines;

            foreach (Part p in sequence.Parts)
            {
                engines.AddRange(p.SubtreeModules.Get<EngineController>().ToArray());
            }
            return engines;
        }
        public ArrayList GetFuelTanks()
        {
            ArrayList tanks = new ArrayList();
            Vehicle? vehicle = Program.ControlledVehicle;
            if (vehicle == null 
                || vehicle.Parts.SequenceList.ActiveSequence < 1
                || vehicle.Parts.SequenceList.Count <= vehicle.Parts.SequenceList.ActiveSequence)
                return tanks;

            Sequence sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.ActiveSequence - 1];
            foreach (Part p in sequence.Parts)
            {
                tanks.AddRange(p.SubtreeModules.Get<Tank>().ToArray());
            }
            return tanks;
        }
        public Tank? GetFuelTank()
        {
            Vehicle? vehicle = Program.ControlledVehicle;
            if (vehicle == null || vehicle.Parts.SequenceList.ActiveSequence < 1) return null;

            Sequence sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.ActiveSequence - 1];
            foreach (Part p in sequence.Parts)
            {
                Span<Tank> tanks = p.SubtreeModules.Get<Tank>();
                if (tanks.Length > 0)
                {
                    Tank tank = tanks[0];
                    return tank;
                }

            }
            return null;
        }
        public bool GetSequenceHasFuel()
        {
            Vehicle? vehicle = Program.ControlledVehicle;

            if (vehicle == null || vehicle.Parts.SequenceList.ActiveSequence < 1) return false;

            ArrayList engines = GetEngineControllers();
            if (engines != null)
            {
                ReadOnlySpan<MoleState> states = vehicle.Parts.Moles.States;
                foreach (EngineController ec in engines)
                {
                    foreach (RocketCore c in ec.Cores)
                    {
                        if (!c.ComputePropellantAvailable(states, false))
                            return false;
                    }
                }
            }
            return vehicle.NavBallData.DeltaV > 0.01;
        }
        
        public double GetAtmosphereHeight()
        {
            if (ControlledVehicle == null
             || ControlledVehicle.Orbit == null
             || ControlledVehicle.Orbit.Parent == null
             || ControlledVehicle.Orbit.Parent.GetAtmosphereReference() == null)
                return 0;

            IParentBody parent = ControlledVehicle.Orbit.Parent;
            #pragma warning disable CS8602 // Dereference of a possibly null reference. Was checked above
            double atmosphereHeight = parent.GetAtmosphereReference().Physical.Height.InMeters();
            return atmosphereHeight;
        }

        public double3 GetOrbitVector()
        {
            Vehicle? vehicle = ControlledVehicle;
            if (vehicle == null) return new double3(0,0,0);

            if (!(vehicle.Orbit.Parent is Celestial celestial))
            {
                return new double3(0,0,0);
            }

            double3 vector = new double3
            {
                Z = celestial.GetAngularVelocity()
            };
            double3 @double = double3.Cross(vector, vehicle.Orbit.StateVectors.PositionCci);
            double3 res = (vehicle.Orbit.StateVectors.VelocityCci - @double);
            res /= res.Length();
            return res;
        }

        public double3 GetSurfaceVector()
        {
            Vehicle? vehicle = ControlledVehicle; 
            if (vehicle == null) return new double3(0, 0, 0);

            if (!(vehicle.Orbit.Parent is Celestial celestial))
            {
                return new double3(0, 0, 0);
            }

            double3 vector = new double3
            {
                Z = celestial.GetAngularVelocity()
            };
            double3 @double = double3.Cross(vector, vehicle.Orbit.StateVectors.PositionCci);
            double3 res = (vehicle.Orbit.StateVectors.VelocityCci - @double);
            res /= res.Length();
            return res;
        }


        // 
        // some stats 
        //
        public double GetAltitude()
        {
            if (ControlledVehicle == null)
                return 0;
            return ControlledVehicle.GetBarometricAltitude();
        }
        public double GetApoapsisAltitude()
        {
            try
            {
                if (Program.GetNearbyCelestial() == null || ControlledVehicle == null)
                    return 0;
                if (ControlledVehicle.Situation == Situation.Landed)
                    return 0;
                return ControlledVehicle.Apoapsis - Program.GetNearbyCelestial().MeanRadius;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return 0;
            }
        }
        public double GetPeriapsisAltitude()
        {
            try
            {
                if (Program.GetNearbyCelestial() == null || ControlledVehicle == null)
                    return 0;
                if (ControlledVehicle.Situation == Situation.Landed)
                    return 0;
                var altPeriapsis = ControlledVehicle.Periapsis - Program.GetNearbyCelestial().MeanRadius;
                return altPeriapsis > 0 ? altPeriapsis : 0;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return 0;
            }
        }

        public double GetApoapsisTime()
        {
            try
            {
                UniverseTime gt = Universe.GetElapsedTime();
                if (ControlledVehicle != null
                    && ControlledVehicle.NextApoapsisTime.IsNotZero()
                    && Program.GetNearbyCelestial() != null
                    && Program.GetNearbyCelestial().GetNearSurfaceRadius() > 0)
                {
                    UniverseTime tta = ControlledVehicle.NextApoapsisTime - gt;
                    if (ControlledVehicle.GetRadarAltitude() < 100 || tta.Seconds() < 0)
                        return 0;
                    else
                        return tta.Seconds();
                }

            }
            catch (Exception)
            {
                return 0;
            }
            return 0;
        }
        public double GetSpeed()
        {
            return ControlledVehicle.GetSurfaceSpeed();
        }

        public double GetRoll()
        {
            double res = Program.AttitudeRoll.Current;
            if (res > 180)
                return res - 360;
            return res;
        }

        public PositionVertex? GetCurrentPosition()
        {
            Vehicle? vehicle = ControlledVehicle;
            if (vehicle == null) return null;
            if (!(vehicle.Orbit.Parent is Celestial celestial))
            {
                return null;
            }
            double3 position = vehicle.Orbit.StateVectors.PositionCci;
            double3 velocity = vehicle.Orbit.StateVectors.VelocityCci;
            double3 surfaceVector = GetSurfaceVector();
            double3 orbitVector = GetOrbitVector();
            return null;
        }

        // The steering direction expressed as the same pitch/heading numbers the in-game
        // navball shows in its surface (EnuBody) frame. Computed with KSA's own functions
        // (ComputeBurnBody2Cci + EnuBody frame + RollPitchYaw decomposition + compass
        // wrap) so the readout matches the navball digit-for-digit. Note KSA's ENU frame
        // is East-referenced, so this differs from a real-world compass azimuth by 90°.
        public static (double pitchDeg, double headingDeg) NavballSteerAngles(double3 r, double3 dir)
        {
            if (r.Length() < 1 || dir.Length() < 1e-9) return (0, 0);

            doubleQuat desired = BurnTarget.ComputeBurnBody2Cci(
                float3.Pack(double3.Normalize(r)), float3.Pack(double3.Normalize(dir)));
            doubleQuat enuBody2Cci = VehicleReferenceFrameEx.GetEnuBody2Cci(r) ?? doubleQuat.Identity;

            // Same construction as the navball: frame -> desired-body orientation.
            doubleQuat frame2Desired = doubleQuat.Concatenate(enuBody2Cci, doubleQuat.Inverse(desired));
            double3 angles = VehicleReferenceFrame.EnuBody.QuaternionToEulerAngles(frame2Desired);

            double pitchDeg = angles.Y * 180.0 / Math.PI;
            double headingDeg = MathEx.ToCompassAngle(angles.Z) * 180.0 / Math.PI;
            return (pitchDeg, headingDeg);
        }

        public static double DegToRad(double degrees)
        {
            // double rad = degrees * Math.PI / 180.0;
            double rad = degrees * (Math.PI / 180.0);

            return rad;
        }

        public static double RadToDeg(double rad)
        {
            //double degrees = rad / (Math.PI / 180.0);
            double degrees = rad * (180.0 /Math.PI);

            return degrees;
        }

    }


    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.PrepareWorker))]
    static class ThrottleOverride
    {
        private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> ManualInputs =
            AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

        public static bool Active;
        public static float Throttle = 1f;
        public static bool EngineOn;

        private static void Prefix(Vehicle __instance)
        {
            if (!Active || __instance != Program.ControlledVehicle)
                return;

            ref ManualControlInputs inputs = ref ManualInputs(__instance);
            inputs.EngineThrottle = Math.Clamp(Throttle, __instance.GetMinThrottle(), 1f);
            inputs.EngineOn = EngineOn;
        }
    }


    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.PrepareWorker))]
    static class FlightControlOverride
    {
        private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> ManualInputs =
            AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

        public static bool Active;
        public static FlightComputerAttitudeMode AttitudeMode = FlightComputerAttitudeMode.Auto;
        public static FlightComputerBurnMode BurnMode = FlightComputerBurnMode.Manual;
        public static VehicleReferenceFrame AttitudeFrame = VehicleReferenceFrame.EnuBody;
        public static FlightComputerAttitudeTrackTarget AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
        public static FlightComputerRollMode RollMode = FlightComputerRollMode.Decoupled;
        public static double3 CustomAttitudeTarget;
        static public FlightComputerRCSMode RCSMode = FlightComputerRCSMode.Enabled;

        public static void SetPitchMode(double pitchDegrees)
        {
            CustomAttitudeTarget = new double3(0, Math.PI * 2 / 360 * -1 * pitchDegrees, 0);
        }

        private static void Prefix(Vehicle __instance)
        {
            if (!Active || __instance != Program.ControlledVehicle)
                return;

            FlightComputer fc = __instance.FlightComputer;
            fc.RollMode = RollMode;
            fc.BurnMode = BurnMode;
            fc.AttitudeMode = AttitudeMode;
            fc.AttitudeFrame = AttitudeFrame;
            fc.CustomAttitudeTarget = CustomAttitudeTarget;
            fc.TrackTarget(AttitudeTrackTarget);
            fc.AttitudeTrackTarget = AttitudeTrackTarget;
            fc.RCSMode = RCSMode;
        }
    }

    [HarmonyPatch(typeof(FlightComputer), "UpdateActiveControlSystems")]
    public class PatchRcsPriority
    {
        public static bool Active;

        static public AttitudeControlSystem PriorityControlSystem = AttitudeControlSystem.Rcs;
        public static void Postfix(FlightComputer __instance, ref readonly FlightComputerOutput outputs)
        {
            if (!Active || __instance != Program.ControlledVehicle?.FlightComputer)
                return;

            if (__instance.RcsTorqueAuthority.X > __instance.TvcTorqueAuthority.X && PriorityControlSystem == AttitudeControlSystem.Rcs)
            {
                __instance.ActiveControlSystem.X = AttitudeControlSystem.Rcs;
                __instance.ActiveControlSystem.Y = AttitudeControlSystem.Tvc;
                __instance.ActiveControlSystem.Z = AttitudeControlSystem.Tvc;
            }
            if (PriorityControlSystem == AttitudeControlSystem.None)
            {
                __instance.ActiveControlSystem.X = AttitudeControlSystem.None;
                __instance.ActiveControlSystem.Y = AttitudeControlSystem.None;
                __instance.ActiveControlSystem.Z = AttitudeControlSystem.None;
            }


        }

    }

}
