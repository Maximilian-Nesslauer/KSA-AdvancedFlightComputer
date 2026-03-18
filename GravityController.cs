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

namespace NovaTec.GravityTurnMod
{
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
        public double InitialPitch { get; set; } = 12.0;
        public double InitialSpeed { get; set; } = 80.0;
        public int TimeToApoapsisStart { get; set; } = 90;
        public int TimeToApoapsisEnd { get; set; } = 90;
        public int TimeToApoapsisTarget { get; set; } = 90;
        public double TargetAltitude { get; set; } = 280.0;
        public double TargetInclination { get; set; } = 0.0;
        public bool UseWarp { get; set; } = true;

        public double DeltaVUsed { get { return _DeltaVUsed; } }
        public enum PhaseEnum
        {
            Landed, Initial, Pitch, Stage, Hold, Coast, Circularize, Idle
        }

        public PhaseEnum Phase = PhaseEnum.Landed;

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

            TimeToApoapsisTarget = TimeToApoapsisStart;
            DeltaVAtLast = vehicle.NavBallData.DeltaVInVacuum;
            DeltaVAtStart = 0;
            _DeltaVUsed = 0;

            Console.WriteLine("Launch vehicle");
            vehicle.SetStabilization(true);
            vehicle.FlightComputer.RateHold(VehicleReferenceFrame.EnuBody); // rate control rel to surface
            vehicle.FlightComputer.RollMode = FlightComputerRollMode.Up;

            Console.WriteLine("Active stage: " + vehicle.Parts.StageList.ActiveStage);
            if (vehicle.Parts.StageList.ActiveStage <= 0)
            {
                NextStage();
            }
            
            // Ignite engines
            vehicle.SetEnum(VehicleEngine.MainIgnite);

            Phase = PhaseEnum.Initial;
            LastTransitionTime = Universe.GetElapsedSimTime();
        }

        public void Run()
        {
            Vehicle? vehicle = ControlledVehicle;
            if (vehicle == null)
                return;

            switch (Phase)
            {
                case PhaseEnum.Landed:
                    RunPhaseLanded(vehicle); break;
                case PhaseEnum.Pitch:
                    RunPhasePitch(vehicle); break;
                case PhaseEnum.Initial:
                    RunPhaseInitial(vehicle); break;
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
            if (GetAltitude() < GetAtmosphereHeight())
            {
                if (vehicle.NavBallData.ThrustWeightRatio > this.DeltaVUsed / 500 + 1.7)
                {
                    ThrottleDown();
                }
                else if (vehicle.NavBallData.ThrustWeightRatio < this.DeltaVUsed / 500 + 1.6 && GetApoapsisTime() < TimeToApoapsisStart)
                {
                    ThrottleUp();
                }
            }

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
                Console.WriteLine("pitch over");
                StartPhasePitch(vehicle);
            }
        }

        public void StartPhasePitch(Vehicle vehicle)   
        {
            Phase = PhaseEnum.Pitch;
            LastTransitionTime = Universe.GetElapsedSimTime();

            vehicle.SetStabilization(true);
            // create custom target for pitch
            double3 target = new double3(Math.PI / 2*0, Math.PI / 2 + Math.PI * 2 / 360 * -1 * InitialPitch, Math.PI / 2*0);
            vehicle.FlightComputer.CustomAttitudeTarget = target;
            vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);

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
            if (UseWarp)
                Universe.SetSimulationSpeed(4.0);

            if (vehicle.FlightComputer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Forward)
                vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Forward);
            LastTransitionTime = Universe.GetElapsedSimTime();
            Phase = PhaseEnum.Hold;
        }

        double lastTimeToApoapsis = 0;
        int tick = 0;
        double fwdPitch = 0;
        bool didReachTargetApoapsis = false;
        private void RunPhaseHold(Vehicle vehicle)
        {
            //CalculateStats();

            tick = (tick++) % 10;
            if (tick != 0) return;


            // needs staging?
            if (vehicle.Parts.StageList.ActiveStage > 0 && !GetStageHasFuel())
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
            if (GetApoapsisTime() < TimeToApoapsisTarget-1 && GetApoapsisTime() > TimeToApoapsisStart && diff < 0)
            {
                double pitch = (double)Program.AttitudePitch.Current;
                if (fwdPitch.IsNearlyZero())
                    fwdPitch = pitch + (TimeToApoapsisTarget - GetApoapsisTime()) / 2;
                //double3 target = new double3(Math.PI / 2 * 0, Math.PI / 2 + Math.PI * 2 / 360 * -1 * 45, 0);
                double3 target = new double3(0, Math.PI * 2 / 360 * -1 * fwdPitch, 0);
                vehicle.FlightComputer.CustomAttitudeTarget = target;
                vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
                ThrottleUp();
            }
            if (GetApoapsisTime() > TimeToApoapsisTarget + 5 && diff > 0)
                TimeToApoapsisTarget = (int)GetApoapsisTime();
            if (GetApoapsisTime() < TimeToApoapsisTarget)
                ThrottleUp();



            // if tta is decreasing with full throttle then pitch up
            if (GetApoapsisTime() < TimeToApoapsisStart - 1 && vehicle.GetManualThrottle() >= 1 && diff < 0)
            {
                double pitch = (double)Program.AttitudePitch.Current;
                if (fwdPitch.IsNearlyZero())
                    fwdPitch = pitch + (TimeToApoapsisStart - GetApoapsisTime())/2;
                //double3 target = new double3(Math.PI / 2*0, Math.PI / 2 + Math.PI * 2 / 360 * -1 * 20, 0);
                double3 target = new double3(0, Math.PI * 2 / 360 * -1 * fwdPitch, 0);
                vehicle.FlightComputer.CustomAttitudeTarget = target;
                vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
            }
            else if (GetApoapsisTime() > (TimeToApoapsisTarget + TimeToApoapsisTarget)/2 && diff > 0)
            {
                fwdPitch = 0;
                if (vehicle.FlightComputer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Forward)
                    vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Forward);
            }

            // transition to coast if AP target altitude is reached.
            if (GetApoapsisAltitude()/1000 > TargetAltitude)
            {
                vehicle.SetEnum(VehicleEngine.MainShutdown);
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
            if (UseWarp)
                Universe.SetSimulationSpeed(1.0);
            LastTransitionTime = Universe.GetElapsedSimTime();
            Phase = PhaseEnum.Stage;
        }
        private void RunPhaseStage(Vehicle vehicle)
        {
            SimTime diff = Universe.GetElapsedSimTime() - LastTransitionTime.Value;

            ThrottleUp();
            // Wait for staging to settle wobble...
            if (vehicle.Parts.StageList.ActiveStage > 0 && !GetStageHasFuel() && diff.Seconds() > 0.5)
            {
                LastTransitionTime = Universe.GetElapsedSimTime();
                Stage stage = NextStage();
            }
            // Wait for ignition to get clear of previous stage
            else if (GetStageHasFuel() && diff.Seconds() > 1.0)
            {
                TimeToApoapsisTarget = (int)GetApoapsisTime();
                StartPhaseHold(vehicle); // back to hold mode
            }

        }
        private void StartPhaseCoast(Vehicle vehicle)
        {
            if (UseWarp)
                Universe.SetSimulationSpeed(4.0);
            LastTransitionTime = Universe.GetElapsedSimTime();
            Phase = PhaseEnum.Coast;

            vehicle.FlightComputer.AttitudeFrame = VehicleReferenceFrame.EclBody;
            vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Prograde);

            vehicle.SetEnum(VehicleEngine.MainShutdown);
        }
        private void RunPhaseCoast(Vehicle vehicle)
        {
            if (GetApoapsisAltitude() / 1000 < TargetAltitude - 5)
            {
                SetEngineThrottle(vehicle, 0.1);
                vehicle.SetEnum(VehicleEngine.MainIgnite);
            }
            else if (GetApoapsisAltitude() / 1000 > TargetAltitude + 5)
            {
                SetEngineThrottle(vehicle, 0.1);
                vehicle.SetEnum(VehicleEngine.MainShutdown);
            }

            if (GetAltitude() > GetAtmosphereHeight())
            {
                StartPhaseCircularize(vehicle);
            }

        }
        public void StartPhaseCircularize(Vehicle vehicle)
        {
            Phase = PhaseEnum.Circularize;

            double currentAp = vehicle.Apoapsis;
            double currentPe = vehicle.Periapsis;
            double targetR = currentAp;

            FlightComputer fc = vehicle.FlightComputer;
            fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
            fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
            fc.AttitudeFrame = VehicleReferenceFrame.EclBody;

            SetEngineThrottle(vehicle, 1.0);
            PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Rcs;

            // create burn to circularize
            double3 dV = OrbitalTransfers.DvCciToCircularize(vehicle.Orbit, vehicle.NextApoapsisTime);
            
            Console.WriteLine("Circularization dV X:" + dV.X + ", Y: " + dV.Y + ", Z: " + dV.Z + ", r2: " + dV.Length() );
            OrbitPointCce point = new OrbitPointCce(vehicle.Orbit.GetApoapsisPositionOrb(), vehicle.TimeSincePeriapsis, vehicle.NextApoapsisTime - Universe.GetElapsedSimTime(), TrueAnomaly.NaN);
            PatchedConic patch = new PatchedConic(vehicle.NextApoapsisTime, vehicle.NextApoapsisTime, PatchTransition.Burn, PatchTransition.Burn, vehicle.Orbit, KeyHash.Make(new ReadOnlySpan<char>("Circularize".ToArray())));
            Burn burn = Burn.Create(point, 
                vehicle.NextApoapsisTime.Seconds(), 
                new double3(dV.Length(),0,0), 
                patch, 
                vehicle);

            fc.AddBurn(burn);

            fc.BurnMode = FlightComputerBurnMode.Auto;
            Universe.WarpToNext();
            
        }

        private void RunPhaseCircularize(Vehicle vehicle)
        {
            if (vehicle.GetManualThrottle() < 1.0 && Universe.GetSimulationSpeed() < 5)
                ThrottleUp();

            FlightComputer fc = vehicle.FlightComputer;
            if (UseWarp)
            {
                if ((Universe.GetElapsedSimTime() - fc.Burn.IgnitionTime).Seconds() > 2 && !Universe.IsAutoWarpActive && Universe.GetSimulationSpeed() < 1.1)
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
            }
            if ((fc.Burn.IgnitionTime - Universe.GetElapsedSimTime()).Seconds() < 3 && Universe.IsAutoWarpActive)
            {
                Universe.AutoWarpStop(true);
            }
            else if (fc.Burn == null || fc.Burn.BurnDuration < 0.001f)
            {
                if (fc.BurnPlan.BurnCount > 0)
                    fc.RemoveBurnAt(0);
                fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
                fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Prograde;
                fc.AttitudeFrame = VehicleReferenceFrame.EclBody;
                Phase = PhaseEnum.Idle;
            }
        }

        private void ThrottleUp()
        {
            ControlledVehicle?.OnKey(new GlfwKeyEvent(Program.GetWindow(), GlfwKeyAction.Press, GlfwKey.Up, 0));
            ControlledVehicle?.PrepareWorker(ControlledVehicle.UpdateTask);
            ControlledVehicle?.OnKey(new GlfwKeyEvent(Program.GetWindow(), GlfwKeyAction.Release, GlfwKey.Up, 0));
        }
        private void ThrottleDown()
        {
            ControlledVehicle?.OnKey(new GlfwKeyEvent(Program.GetWindow(), GlfwKeyAction.Press, GlfwKey.Down, 0));
            ControlledVehicle?.PrepareWorker(ControlledVehicle.UpdateTask);
            ControlledVehicle?.OnKey(new GlfwKeyEvent(Program.GetWindow(), GlfwKeyAction.Release, GlfwKey.Down, 0));
        }
        private void SetEngineThrottle(Vehicle vehicle, double throttleValue)
        {
            FieldInfo? manualControlInputsField = typeof(Vehicle).GetField("_manualControlInputs", BindingFlags.NonPublic | BindingFlags.Instance);
            if (manualControlInputsField == null) return;

            object? controlInputs = manualControlInputsField.GetValue(vehicle);
            if (controlInputs == null) return;

            PropertyInfo? engineThrottleProp = controlInputs.GetType().GetProperty("EngineThrottle");
            FieldInfo? engineThrottleField = controlInputs.GetType().GetField("EngineThrottle");

            float clampedValue = (float)Math.Clamp(throttleValue, 0.0, 1.0);

            if (engineThrottleProp != null)
            {
                engineThrottleProp.SetValue(controlInputs, clampedValue);
            }
            else if (engineThrottleField != null)
            {
                engineThrottleField.SetValue(controlInputs, clampedValue);
            }
        }

        //-------------------------------------------------------------------------

        public void CalculateStats()
        {
            if (ControlledVehicle == null || GetCurrentStage() == null || !GetCurrentStage().ContainsEngine || ControlledVehicle.NavBallData.DeltaVInVacuum <= 0)
                return;

            if (DeltaVAtLast < ControlledVehicle.NavBallData.DeltaVInVacuum)
                DeltaVAtLast = ControlledVehicle.NavBallData.DeltaVInVacuum;


            _DeltaVUsed += DeltaVAtLast - ControlledVehicle.NavBallData.DeltaVInVacuum;
            DeltaVAtLast = ControlledVehicle.NavBallData.DeltaVInVacuum;
        }
        public Stage GetCurrentStage()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            Stage stage = null;

            if (vehicle.Parts.StageList.ActiveStage > 0)
            {
                stage = vehicle.Parts.StageList.Stages[vehicle.Parts.StageList.ActiveStage - 1];
            }
            return stage;
        }
        public Stage NextStage()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            Stage stage = null;

            //Console.WriteLine("Stages: " + vehicle.Parts.StageList.Stages.Length);
            if (vehicle.Parts.StageList.ActiveStage < 0)
            {
                stage = vehicle.Parts.StageList.Stages[vehicle.Parts.StageList.Stages.Length - 1];
                vehicle.Parts.StageList.SetActiveStage(stage.StageNumber);
            }
            else if (vehicle.Parts.StageList.ActiveStage > 0)
            {
                vehicle.SetEnum(VehicleEngine.MainShutdown);
                vehicle.Parts.StageList.ActivateNextStage(vehicle);
            }
            if (vehicle.Parts.StageList.ActiveStage > 0)
            {
                stage = vehicle.Parts.StageList.Stages[vehicle.Parts.StageList.Stages.Length - 1];
                Console.WriteLine("Activated stage: " + vehicle.Parts.StageList.ActiveStage);
                if (stage.ContainsEngine)
                    vehicle.SetEnum(VehicleEngine.MainIgnite);
            }

            return stage;
        }

        public float GetFuelInStage()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            Stage stage = vehicle.Parts.StageList.Stages[vehicle.Parts.StageList.ActiveStage - 1];
            foreach (Part p in stage.Parts)
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
            if (vehicle == null || vehicle.Parts.StageList.ActiveStage < 1) return null;

            Stage stage = vehicle.Parts.StageList.Stages[vehicle.Parts.StageList.ActiveStage - 1];
            ArrayList engines = new ArrayList();
            foreach (Part p in stage.Parts)
            {
                engines.AddRange(p.SubtreeModules.Get<EngineController>().ToArray());
            }
            return engines;
        }
        public ArrayList GetFuelTanks()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            if (vehicle == null || vehicle.Parts.StageList.ActiveStage < 1) return null;

            Stage stage = vehicle.Parts.StageList.Stages[vehicle.Parts.StageList.ActiveStage - 1];
            ArrayList tanks = new ArrayList();
            foreach (Part p in stage.Parts)
            {
                tanks.AddRange(p.SubtreeModules.Get<Tank>().ToArray());
            }
            return tanks;
        }
        public Tank GetFuelTank()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            if (vehicle == null || vehicle.Parts.StageList.ActiveStage < 1) return null;

            Stage stage = vehicle.Parts.StageList.Stages[vehicle.Parts.StageList.ActiveStage - 1];
            foreach (Part p in stage.Parts)
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
        public bool GetStageHasFuel()
        {
            Vehicle vehicle = Program.ControlledVehicle;
            if (vehicle == null || vehicle.Parts.StageList.ActiveStage < 1) return false;
            bool hasFuel = false;
            ArrayList engines = GetEngineControllers();
            if (engines != null)
            {
                ReadOnlySpan<MoleState> states = vehicle.Parts.Moles.States;
                foreach (EngineController ec in engines)
                {
                    foreach (RocketCore c in ec.Cores)
                    {
                        hasFuel |= c.ResourceManager.ResourceAvailable(states);
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
}
