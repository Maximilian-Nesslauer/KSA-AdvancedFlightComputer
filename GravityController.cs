using Brutal.GlfwApi;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using RenderCore.Input;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.Text;
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
        public double InitialPitch { get; set; } = 10.0;
        public double InitialSpeed { get; set; } = 80.0;
        public int TimeToApoapsisStart { get; set; } = 65;
        public int TimeToApoapsisEnd { get; set; } = 65;
        public int TimeToApoapsisTarget { get; set; } = 70;
        public double TargetAltitude { get; set; } = 280.0;
        
        /* works for 1/4th size
        public double InitialPitch { get; set; } = 12.0;
        public double InitialSpeed { get; set; } = 70.0;
        public int TimeToApoapsisStart { get; set; } = 50;
        public int TimeToApoapsisEnd { get; set; } = 50;
        public int TimeToApoapsisTarget { get; set; } = 50;
        public double TargetAltitude { get; set; } = 120.0;
        */

        public double TargetInclination { get; set; } = 0.0;
        public bool UseWarp { get; set; } = true;
        public bool AutoStage{ get; set; } = true;

        public double DeltaVUsed { get { return _DeltaVUsed; } }
        public bool PitchesUp{ get; set; } = false;
        public enum PhaseEnum
        {
            Landed, Initial, Pitch, Stage, Hold, Coast, Circularize, Idle
        }

        public PhaseEnum Phase = PhaseEnum.Landed;
        public long tick = 0;

        private Vehicle? ControlledVehicle = null;

        private double LaunchAltitude = 0.0;

        private SimTime? LastTransitionTime = null;

        private double DeltaVAtStart = 0.0;
        private double DeltaVAtLast = 0.0;
        private double _DeltaVUsed = 0.0;

        public GravityController(Vehicle vehicle) 
        {
            Phase = PhaseEnum.Landed;
            ControlledVehicle = vehicle;
        }

        public void Launch(Vehicle vehicle)
        {
            if (vehicle == null)
                return;

            ControlledVehicle = vehicle;
            LaunchAltitude = GetAltitude();
            LaunchAltitude = ControlledVehicle.Apoapsis;
            tick = System.DateTime.Now.Ticks;

            TimeToApoapsisTarget = TimeToApoapsisStart;
            DeltaVAtLast = vehicle.NavBallData.DeltaV;
            DeltaVAtStart = 0;
            _DeltaVUsed = 0;

            Console.WriteLine("Launch vehicle");
            vehicle.SetStabilization(true);
/*            vehicle.FlightComputer.RateHold(VehicleReferenceFrame.EnuBody); // rate control rel to surface
            vehicle.FlightComputer.RollMode = FlightComputerRollMode.Up;
            vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Up);
            vehicle.FlightComputer.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Up;
*/
            FlightControlOverride.Active = true;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
            FlightControlOverride.RollMode = vehicle.FlightComputer.RollMode = FlightComputerRollMode.Up;
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
            LastTransitionTime = Universe.GetElapsedSimTime();
        }

        public void Run()
        {
            Vehicle? vehicle = ControlledVehicle;
            if (vehicle == null)
                return;

            //if (System.DateTime.Now.Ticks - tick < 50*10000 && tick != 0) return;
            tick = System.DateTime.Now.Ticks;

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
            }
                        
                                    // hack to simulate lower TWR engines
                                    if (GetAtmosphereHeight() > 10 && GetAltitude() < GetAtmosphereHeight()/3 && (Phase != PhaseEnum.Landed && Phase != PhaseEnum.Idle))
                                    {
                                        if (vehicle.NavBallData.ThrustWeightRatio > this.DeltaVUsed / 600 + 1.75 || vehicle.NavBallData.ThrustWeightRatio > 3)
                                        {
                                            ThrottleDown();
                                        }
                                        //else if (vehicle.NavBallData.ThrustWeightRatio < this.DeltaVUsed / 600 + 1.65 && GetApoapsisTime() < TimeToApoapsisStart)
                                        //{
                                        //    ThrottleUp();
                                        //}
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
            //CalculateStats();

            if (GetSpeed() > InitialSpeed)
            {
                StartPhasePitch(vehicle);
            }
        }

        public void StartPhasePitch(Vehicle vehicle)   
        {
            Console.WriteLine("PHASE: Pitch over");

            Phase = PhaseEnum.Pitch;
            LastTransitionTime = Universe.GetElapsedSimTime();

            vehicle.SetStabilization(true);
            // create custom target for pitch
/*            double3 target = new double3(Math.PI / 2*0, Math.PI / 2 + Math.PI * 2 / 360 * -1 * InitialPitch, Math.PI / 2*0);
            vehicle.FlightComputer.CustomAttitudeTarget = target;
            vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
            vehicle.FlightComputer.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
*/
            double3 target = new double3(Math.PI / 2 * 0, Math.PI / 2 + Math.PI * 2 / 360 * -1 * InitialPitch, Math.PI / 2 * 0);
            FlightControlOverride.Active = true;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
            FlightControlOverride.CustomAttitudeTarget = target;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
            FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EnuBody;

        }

        private void RunPhasePitch(Vehicle vehicle)
        {
            //CalculateStats();
            double pitch = vehicle.BodyRates.X;

            SimTime diff = Universe.GetElapsedSimTime() - LastTransitionTime.Value;
            if (UseWarp && diff.Seconds() > 3 && Universe.SimulationSpeed < 1.1)
                Universe.SetSimulationSpeed(2.0);

            if (diff.Seconds() > InitialPitch * 0.8 /*&& Math.Abs(vehicle.BodyRates.X) < 0.005 && Math.Abs(vehicle.BodyRates.Y) < 0.005*/)
            {
                StartPhaseHold(vehicle);
            }
        }
        public void StartPhaseHold(Vehicle vehicle)
        {
            FlightControlOverride.Active = false;
            Console.WriteLine("PHASE: Hold");
            if (UseWarp)
                Universe.SetSimulationSpeed(4.0);

            /*            if (vehicle.FlightComputer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Forward)
                            vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Forward);
                        vehicle.FlightComputer.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Forward;
                        RunWorker();
            */
            FlightControlOverride.Active = true;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Forward;
            FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EnuBody;

            LastTransitionTime = Universe.GetElapsedSimTime();
            Phase = PhaseEnum.Hold;
        }

        double lastTimeToApoapsis = 0;
        double fwdPitch = 0;
        double iniPitch = 0;

        bool didReachTargetApoapsis = false;

        private void RunPhaseHold(Vehicle vehicle)
        {
            CalculateStats();

            // needs staging?
            if (vehicle.Parts.SequenceList.ActiveSequence > 0 && !GetSequenceHasFuel())
            {
                StartPhaseStage(vehicle);
                return;
            }

            // roll is at nearly 0?
            if (Math.Abs(GetRoll()) < 6 && vehicle.FlightComputer.ActiveControlSystem.X == AttitudeControlSystem.Rcs)
                PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Tvc;
            else if (Math.Abs(GetRoll()) > 6 && vehicle.FlightComputer.ActiveControlSystem.X != AttitudeControlSystem.Rcs)
                PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Rcs;

            SimTime dt = Universe.GetElapsedSimTime() - LastTransitionTime.Value;

            double diff = GetApoapsisTime() - lastTimeToApoapsis;

            // now check time to apoapsis
            if (GetApoapsisTime() > TimeToApoapsisTarget && vehicle.GetManualThrottle() > 0.4 && dt > 5)
            {
                didReachTargetApoapsis = true;
                ThrottleDown();
            }

            // pitch up or down based on difference to target AP time
            if (GetApoapsisTime() > TimeToApoapsisTarget + 5 && diff > 0)
                TimeToApoapsisTarget = (int)GetApoapsisTime();
            if (GetApoapsisTime() < TimeToApoapsisTarget)
                ThrottleUp();

            // after 1st stage sequency do pitch up or down. If that is not enough, it's an indicator of wrong startup values.
            // In general the need to pitch up is a sign of a weak 2nd stage.
            if (vehicle.Parts.SequenceList.ActiveSequence > 2 && didReachTargetApoapsis)
            {
/*                if (GetApoapsisTime() < TimeToApoapsisTarget - 1 && GetApoapsisTime() > TimeToApoapsisStart-2 && diff < 0)
                {
                    double pitch = (double)Program.AttitudePitch.Current;
                    if (iniPitch.IsNearlyZero())
                        iniPitch = pitch;
                    else
                        pitch = iniPitch;

                    if ((TimeToApoapsisTarget - GetApoapsisTime()) / 2 < 0)
                        fwdPitch = pitch + (TimeToApoapsisTarget - GetApoapsisTime()) / 2;
                    else
                        fwdPitch = pitch;

                    //double3 target = new double3(Math.PI / 2 * 0, Math.PI / 2 + Math.PI * 2 / 360 * -1 * 45, 0);
                    double3 target = new double3(0, Math.PI * 2 / 360 * -1 * fwdPitch, 0);
                    vehicle.FlightComputer.CustomAttitudeTarget = target;
                    if (vehicle.FlightComputer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Custom)
                    {
                        vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
                        RunWorker();
                    }
                    //vehicle.FlightComputer.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
                    PitchesUp = true;
                    Universe.SetSimulationSpeed(1.0, false);
                }

                // if tta is decreasing with full throttle then pitch up
                else*/ if (GetApoapsisTime() < TimeToApoapsisStart - 1 && vehicle.GetManualThrottle() >= 1 && diff < 0)
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

                    //double3 target = new double3(Math.PI / 2*0, Math.PI / 2 + Math.PI * 2 / 360 * -1 * 20, 0);
                    double3 target = new double3(0, Math.PI * 2 / 360 * -1 * fwdPitch, 0);
                    vehicle.FlightComputer.CustomAttitudeTarget = target;
                    if (vehicle.FlightComputer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Custom)
                    {
                        vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
                        RunWorker();
                    }
                    PitchesUp = true;
                }
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

                    //double3 target = new double3(Math.PI / 2*0, Math.PI / 2 + Math.PI * 2 / 360 * -1 * 20, 0);
                    double3 target = new double3(0, Math.PI * 2 / 360 * -1 * fwdPitch, 0);
                    vehicle.FlightComputer.CustomAttitudeTarget = target;
                    if (vehicle.FlightComputer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Custom)
                    {
                        vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
                        RunWorker();
                    }
                    PitchesUp = true;
                }
                else if (GetApoapsisTime() > (TimeToApoapsisTarget + TimeToApoapsisTarget) / 2 && diff > 0)
                {
                    iniPitch = 0;
                    fwdPitch = 0;
                    if (vehicle.FlightComputer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Forward)
                    {
                        vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Forward);
                        RunWorker();
                    }

                    PitchesUp = false;
                }
            }

            // transition to coast if AP target altitude is reached.
            if (GetApoapsisAltitude()/1000 > TargetAltitude)
            {
                ShutdownEngines();
                StartPhaseCoast(vehicle);
            }

            // increase sim speed if AP is above atmosphere
            if (GetApoapsisAltitude() > GetAtmosphereHeight() && UseWarp)
            {
                Universe.SetSimulationSpeed(10.0);
            }

            lastTimeToApoapsis = GetApoapsisTime();

        }

        private void StartPhaseStage(Vehicle vehicle)
        {
            Console.WriteLine("PHASE: Stage");
            if (UseWarp)
                Universe.SetSimulationSpeed(1.0);
            LastTransitionTime = Universe.GetElapsedSimTime();
            Phase = PhaseEnum.Stage;
        }
        private void RunPhaseStage(Vehicle vehicle)
        {
            SimTime diff = Universe.GetElapsedSimTime() - LastTransitionTime.Value;

            SetEngineThrottle(1.0);
            // Wait for staging to settle wobble...
            if (vehicle.Parts.SequenceList.ActiveSequence > 0 && !GetSequenceHasFuel() && diff.Seconds() > 0.3)
            {
                LastTransitionTime = Universe.GetElapsedSimTime();
                NextStequence();
            }
            // Wait for ignition to get clear of previous stage
            else if (diff.Seconds() > 0.8)
            {
                // no fuel in this stage, then it's probably a decoupler, so skip to next stage
                if (!GetSequenceHasFuel())
                    NextStequence();

                if ((int)GetApoapsisTime() > TimeToApoapsisStart)
                    TimeToApoapsisTarget = (int)GetApoapsisTime();
                StartPhaseHold(vehicle); // back to hold mode
            }

        }
        private void StartPhaseCoast(Vehicle vehicle)
        {
            Console.WriteLine("PHASE: Coast");
            if (UseWarp)
                Universe.SetSimulationSpeed(4.0);
            LastTransitionTime = Universe.GetElapsedSimTime();
            Phase = PhaseEnum.Coast;

            FlightControlOverride.Active = true;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
            FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EclBody;
            FlightControlOverride.RollMode = FlightComputerRollMode.Up;

            ShutdownEngines();
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

            if (GetAltitude() > GetAtmosphereHeight())
            {
                StartPhaseCircularize(vehicle);
            }

        }
        public void StartPhaseCircularize(Vehicle vehicle)
        {
            Console.WriteLine("PHASE: Circularize");
            Phase = PhaseEnum.Circularize;
            ThrottleOverride.Active = false;
            FlightControlOverride.Active = false;
            ControlledVehicle?.FlightComputer.BurnMode = FlightComputerBurnMode.Auto;

            double currentAp = vehicle.Apoapsis;
            double currentPe = vehicle.Periapsis;
            double targetR = currentAp;



            FlightComputer fc = vehicle.FlightComputer;

            ThrottleOverride.Active = true;
            SetEngineThrottle(1.0);
            PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Rcs;
            FlightControlOverride.Active = true;
            FlightControlOverride.BurnMode = FlightComputerBurnMode.Auto;
            FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
            FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EclBody;

            // create burn to circularize
            double3 dV = OrbitalTransfers.DvCciToCircularize(vehicle.Orbit, vehicle.NextApoapsisTime);

            Console.WriteLine("Circularization dV X:" + dV.X + ", Y: " + dV.Y + ", Z: " + dV.Z + ", r2: " + dV.Length());
            OrbitPointCce point = new OrbitPointCce(vehicle.Orbit.GetApoapsisPositionOrb(), vehicle.TimeSincePeriapsis, vehicle.NextApoapsisTime - Universe.GetElapsedSimTime(), TrueAnomaly.NaN);
            PatchedConic patch = new PatchedConic(vehicle.NextApoapsisTime, vehicle.NextApoapsisTime, PatchTransition.Burn, PatchTransition.Burn, vehicle.Orbit, KeyHash.Make(new ReadOnlySpan<char>("Circularize".ToArray())));
            Burn burn = Burn.Create(point,
                vehicle.NextApoapsisTime.Seconds(),
                new double3(dV.Length()*1.0, 0, 0),
                patch,
                vehicle);

            fc.AddBurn(burn);

            if (UseWarp)
            {
                Universe.SetSimulationSpeed(10.0, false);
                Universe.WarpToNext();
            }
        }

        private void RunPhaseCircularize(Vehicle vehicle)
        {
            if (vehicle.GetManualThrottle() < 1.0 && Universe.GetSimulationSpeed() < 5)
                ThrottleUp();


            FlightComputer fc = vehicle.FlightComputer;
            if (UseWarp)
            {
/*                if ((Universe.GetElapsedSimTime() - fc.Burn.IgnitionTime).Seconds() >= 3 && !Universe.IsAutoWarpActive && Universe.GetSimulationSpeed() < 1.1)
                {
                    if (fc.Burn.BurnDuration > 10)
                        Universe.SetSimulationSpeed(2.0, false);
                    if (fc.Burn.BurnDuration > 20)
                        Universe.SetSimulationSpeed(4.0, false);
                }
                else if (fc.Burn.BurnDuration < 3 && !Universe.IsAutoWarpActive && Universe.GetSimulationSpeed() > 1.1)
                {
                    Universe.SetSimulationSpeed(1.0, false);
                }
*/
            }
            // Burn is about to start, so stop auto warp and set speed to 1x
            if ((fc.Burn.IgnitionTime - Universe.GetElapsedSimTime()).Seconds() < 10 && (Universe.IsAutoWarpActive || Universe.GetSimulationSpeed() > 1))
            {
                ThrottleOverride.Active = false;
                FlightControlOverride.Active = false;

                Console.WriteLine("Burn close to ignition");
                Console.WriteLine("  Burns: {0}", fc.BurnPlan.BurnCount);
                Console.WriteLine("  Duration: {0}", fc.Burn.BurnDuration);
                Universe.SetSimulationSpeed(1.0, false);
                Universe.AutoWarpStop(true);
            }
            // burn completed?
            else if (fc.Burn == null || fc.Burn.BurnDuration < 0.1f)
            {
                Console.WriteLine("Burn complete? Burns: {0}", fc.BurnPlan.BurnCount);
                Console.WriteLine("  Duration: {0}", fc.Burn.BurnDuration);
                if (fc.BurnPlan.BurnCount > 0)
                {
                    fc.RemoveBurnAt(fc.BurnPlan.BurnCount-1);
                    Console.WriteLine("Burn complete, deleting burn");
                }
                FlightControlOverride.Active = true;
                FlightControlOverride.BurnMode = FlightComputerBurnMode.Manual;
                FlightControlOverride.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
                FlightControlOverride.AttitudeFrame = VehicleReferenceFrame.EclBody;
                FlightControlOverride.AttitudeMode = FlightComputerAttitudeMode.Auto;

                Phase = PhaseEnum.Idle;
            }
            else if ((fc.Burn.IgnitionTime - Universe.GetElapsedSimTime()).Seconds() < 0)
            {
                Console.WriteLine("Burns: {0}", fc.BurnPlan.BurnCount);
                Console.WriteLine("   Burn Duration: {0}", fc.Burn.BurnDuration);
                // needs staging?
                if (vehicle.Parts.SequenceList.ActiveSequence > 0 && !GetSequenceHasFuel() && AutoStage)
                {
                    Console.WriteLine("Burn complete, stage!");
                    StartPhaseStage(vehicle);
                }
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
            SetEngineThrottle(vehicle.GetManualThrottle() + 0.01);
        }
        public void ThrottleDown()
        {
            var vehicle = ControlledVehicle;
            if (vehicle is null) return;
            SetEngineThrottle(vehicle.GetManualThrottle() - 0.01);
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
            ControlledVehicle?.SetEnum(VehicleEngine.MainIgnite);
        }
        public void ShutdownEngines()
        {
            ThrottleOverride.EngineOn = false;
            ControlledVehicle?.SetEnum(VehicleEngine.MainShutdown);
            ThrottleOverride.Active = false;
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
        public Sequence GetCurrentSequence()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            Sequence sequence = null;

            if (vehicle != null && vehicle.Parts != null && vehicle.Parts.SequenceList != null 
                && vehicle.Parts.SequenceList.ActiveSequence > 0
                && vehicle.Parts.SequenceList.Count > vehicle.Parts.SequenceList.ActiveSequence)
            {
                sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.ActiveSequence - 1];
            }
            return sequence;
        }
        public Sequence NextStequence()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            Sequence sequence = null;

            Console.WriteLine("Sequence: {0:D} / {1:D}", vehicle.Parts.SequenceList.ActiveSequence, vehicle.Parts.SequenceList.Count );
            if (vehicle != null && vehicle.Parts.SequenceList.ActiveSequence <= 0)
            {
                //sequence = vehicle.Parts.SequenceList.Sequences[0];
                //vehicle.Parts.SequenceList.SetActiveSequence(sequence.Number);
                vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            }
            else if (vehicle.Parts.SequenceList.ActiveSequence > 0)
            {
                ShutdownEngines();
                vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
                //RunWorker();
            }

            //vehicle.Parts.SequenceList.ResetCaches();
            //GetCurrentSequence().RecacheParts();
            vehicle.UpdateAfterPartTreeModification();

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
            Vehicle vehicle = Program.ControlledVehicle;
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
            Vehicle vehicle = Program.ControlledVehicle;
            if (vehicle == null || vehicle.Parts.SequenceList.ActiveSequence < 1) return null;

            Sequence sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.ActiveSequence - 1];
            ArrayList engines = new ArrayList();
            foreach (Part p in sequence.Parts)
            {
                engines.AddRange(p.SubtreeModules.Get<EngineController>().ToArray());
            }
            return engines;
        }
        public ArrayList GetFuelTanks()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            if (vehicle == null 
                || vehicle.Parts.SequenceList.ActiveSequence < 1
                || vehicle.Parts.SequenceList.Count <= vehicle.Parts.SequenceList.ActiveSequence)
                return null;

            Sequence sequence = vehicle.Parts.SequenceList.Sequences[vehicle.Parts.SequenceList.ActiveSequence - 1];
            ArrayList tanks = new ArrayList();
            foreach (Part p in sequence.Parts)
            {
                tanks.AddRange(p.SubtreeModules.Get<Tank>().ToArray());
            }
            return tanks;
        }
        public Tank GetFuelTank()
        {
            Vehicle vehicle = Program.ControlledVehicle;
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
            Vehicle vehicle = Program.ControlledVehicle;

            if (vehicle == null || vehicle.Parts.SequenceList.ActiveSequence < 1) return false;

            bool hasFuel = false;
            ArrayList engines = GetEngineControllers();
            if (engines != null)
            {
                ReadOnlySpan<MoleState> states = vehicle.Parts.Moles.States;
                foreach (EngineController ec in engines)
                {
                    foreach (RocketCore c in ec.Cores)
                    {
                        hasFuel |= c.ComputePropellantAvailable(states, true);
                    }
                }
            }
            return hasFuel;
        }
        
        public double GetAtmosphereHeight()
        {
            if (ControlledVehicle == null
             || ControlledVehicle.Orbit == null
             || ControlledVehicle.Orbit.Parent == null
             || ControlledVehicle.Orbit.Parent.GetAtmosphereReference() == null)
                return 0;

            IParentBody parent = ControlledVehicle.Orbit.Parent;
            double atmosphereHeight = parent.GetAtmosphereReference().Physical.Height.InMeters();
            return atmosphereHeight;
        }

        public double3 GetOrbitVector()
        {
            Vehicle vehicle = ControlledVehicle;
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
            Vehicle vehicle = ControlledVehicle; 
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
                return ControlledVehicle.Apoapsis - Program.GetNearbyCelestial().MeanRadius;
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
                SimTime gt = Universe.GetElapsedSimTime();
                if (ControlledVehicle != null
                    && ControlledVehicle.NextApoapsisTime.IsNotNaN()
                    && ControlledVehicle.NextApoapsisTime.IsNotZero()
                    && Program.GetNearbyCelestial() != null
                    && Program.GetNearbyCelestial().GetNearSurfaceRadius() != null)
                {
                    SimTime tta = ControlledVehicle.NextApoapsisTime - gt;
                    if (GetAltitude() < 100)
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
        public static FlightComputerRollMode RollMode = FlightComputerRollMode.Up;
        public static double3 CustomAttitudeTarget;

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
        }
    }

    [HarmonyPatch(typeof(FlightComputer), "UpdateActiveControlSystems")]
    public class PatchRcsPriority
    {
        static public AttitudeControlSystem PriorityControlSystem = AttitudeControlSystem.Rcs;
        public static void Postfix(FlightComputer __instance, ref readonly FlightComputerOutput outputs)
        {
            if (__instance.RcsTorqueAuthority.X > __instance.TvcTorqueAuthority.X && PriorityControlSystem == AttitudeControlSystem.Rcs)
                __instance.ActiveControlSystem.X = AttitudeControlSystem.Rcs;
        }
    }

}
