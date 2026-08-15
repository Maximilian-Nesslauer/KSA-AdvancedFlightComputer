using Brutal.GlfwApi;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using RenderCore.Input;
using StarMap.API;
using System.Collections;
using static Brutal.Strings.Utf8;

namespace NovaTec.GravityTurnMod
{

    [StarMapMod]
    public class GravityTurn
    {
        private GravityController? Controller = null;

        public String CoordToString(doubleQuat q)
        {
            double3 angles = VehicleReferenceFrameEx.QuaternionToEulerAngles(VehicleReferenceFrame.EnuBody, q);

            return GravityController.CoordToString(angles);
        }

        [StarMapAfterGui]
        public void OnAfterUi(double dt)
        {

            Vehicle? vehicle = Program.ControlledVehicle;
            if (vehicle == null)
                return;

            if (Controller == null)
                Controller = new GravityController(vehicle);
            else
                Controller.Run();

            ImGuiWindowFlags flags = ImGuiWindowFlags.MenuBar;

            if (ImGui.IsKeyPressed(ImGuiKey.F11))
            {
                ImGui.Text("TVC control");

                PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Tvc;
            }
            if (ImGui.IsKeyPressed(ImGuiKey.F12))
            {
                ImGui.Text("RCS control");

                PatchRcsPriority.PriorityControlSystem = AttitudeControlSystem.Rcs;
            }

            ImGui.Begin("Gravityturn", flags);
            ImGui.SetWindowSize("Gravityturn", new Brutal.Numerics.float2(600, 620));

            if (ImGui.BeginMenuBar())
            {
                if (ImGui.BeginMenu("Control"))
                {
                    if (ImGui.MenuItem("Launch"))
                    {
                        Controller.Launch(vehicle);
                        Controller.Phase = GravityController.PhaseEnum.Landed;
                    }
                    if (ImGui.MenuItem("Pitch over"))
                    {
                        Controller.StartPhasePitch(vehicle);
                        Controller.Phase = GravityController.PhaseEnum.Landed;
                    }
                    if (ImGui.MenuItem("Stability Assist"))
                    {
                        Console.WriteLine("toggle Stability Assist");
                        vehicle.ToggleStabilization();
                    }
                    if (ImGui.MenuItem("RCS"))
                    {
                        Console.WriteLine("toggle RCS");
                        FlightControlOverride.Active = true;
                        FlightControlOverride.RCSMode = FlightControlOverride.RCSMode == FlightComputerRCSMode.Disabled ? FlightComputerRCSMode.Enabled : FlightComputerRCSMode.Disabled;
                    }
                    if (ImGui.MenuItem("Stage"))
                    {
                        Controller.NextStequence();
                    }
                    if (ImGui.MenuItem("Throttle UP"))
                    {
                        Controller.ThrottleUp();
                    }
                    if (ImGui.MenuItem("Throttle DOWN"))
                    {
                        Controller.ThrottleDown();
                    }

                    if (ImGui.MenuItem("HOLD Forward/Prograde"))
                    {
                        Controller.StartPhaseHold(vehicle);
                        Controller.Phase = GravityController.PhaseEnum.Idle;
                    }
                    if (ImGui.MenuItem("Circularize"))
                    {
                        Controller.StartPhaseCircularize(vehicle);
                    }
                    if (ImGui.MenuItem("Go IDLE"))
                    {
                        Controller.Phase = GravityController.PhaseEnum.Idle;
                    }
                    ImGui.EndMenu();
                }
                ImGui.EndMenuBar();
            }

            double pdt = KSA.Program.GetPlayerDeltaTime();
            double pt = KSA.Program.GetPlayerTime();
            UniverseTime gt = Universe.GetElapsedTime();
            if (vehicle != null 
                && vehicle.NextApoapsisTime.IsNotZero()
                && Program.GetNearbyCelestial() != null
               )
            {
                Controller.CalculateStats();
                float width = ImGui.GetWindowSize().X;
                //float width = ImGui.CalcItemWidth();
                float x = ImGui.GetCursorPosX();
                float y = ImGui.GetCursorPosY();

                // setup input parameters

                ImGui.Text("Target altitude");
                ImGui.SameLine();
                ImGui.SetCursorPosX(x + width * 0.45f + ImGui.GetStyle().ItemInnerSpacing.X);
                ImGui.SetNextItemWidth(width * 0.2f);
                int talt = (int)Controller.TargetAltitude;
                ImGui.InputInt("km", flags: ImGuiInputTextFlags.CharsDecimal, v: ref talt);
                if (talt != Controller.TargetAltitude)
                {
                    Controller.TargetAltitude = talt;
                }

				ImGui.Text("Target inclination");
				ImGui.SameLine();
				ImGui.SetCursorPosX(x + width * 0.45f + ImGui.GetStyle().ItemInnerSpacing.X);
				ImGui.SetNextItemWidth(width * 0.2f);
				int tinc= (int)Controller.TargetInclination;
				ImGui.InputInt("°", flags: ImGuiInputTextFlags.CharsDecimal, v: ref tinc);
				if (tinc != Controller.TargetInclination)
				{
					Controller.TargetInclination = tinc;
				}
				
                ImGui.Text("Pitch speed");
                ImGui.SameLine();
                ImGui.SetCursorPosX(x + width * 0.45f + ImGui.GetStyle().ItemInnerSpacing.X);
                ImGui.SetNextItemWidth(width * 0.2f);
                double initialSpeed = Controller.InitialSpeed;
                ImGui.InputDouble("m/s", format: "%.1f", flags: ImGuiInputTextFlags.CharsDecimal,v: ref initialSpeed);
                if (initialSpeed != Controller.InitialSpeed)
                    Controller.InitialSpeed = initialSpeed;

                ImGui.Text("Pitch angle");
                ImGui.SameLine();
                ImGui.SetCursorPosX(x + width * 0.45f + ImGui.GetStyle().ItemInnerSpacing.X);
                ImGui.SetNextItemWidth(width * 0.2f);
                double initialAngle = Controller.InitialPitch;
                ImGui.InputDouble("° ", format: "%.1f", flags: ImGuiInputTextFlags.CharsDecimal, v: ref initialAngle);
                if (initialAngle != Controller.InitialPitch)
                    Controller.InitialPitch = initialAngle;

                ImGui.Text("Time to apoapsis start");
                ImGui.SameLine();
                ImGui.SetCursorPosX(x + width * 0.45f + ImGui.GetStyle().ItemInnerSpacing.X);
                ImGui.SetNextItemWidth(width * 0.2f);
                int ttas = Controller.TimeToApoapsisStart;
                ImGui.InputInt("s", flags: ImGuiInputTextFlags.CharsDecimal, v: ref ttas);
                if (ttas != Controller.TimeToApoapsisStart)
                {
                    Controller.TimeToApoapsisStart = ttas;
                    Controller.TimeToApoapsisEnd = ttas;
                }
                
                ImGui.Text("Use time warp:");
                ImGui.SameLine();
                ImGui.SetCursorPosX(x + width * 0.45f + ImGui.GetStyle().ItemInnerSpacing.X);
                ImGui.SetNextItemWidth(width * 0.2f);
                bool uw = Controller.UseWarp;
                ImGui.Checkbox("##tw", ref uw);
                if (uw != Controller.UseWarp)
                {
                    Controller.UseWarp = uw;    
                }

                ImGui.Text("Auto stage:");
                ImGui.SameLine();
                ImGui.SetCursorPosX(x + width * 0.45f + ImGui.GetStyle().ItemInnerSpacing.X);
                ImGui.SetNextItemWidth(width * 0.2f);
                bool ast = Controller.AutoStage;
                ImGui.Checkbox("##as", ref ast);
                if (ast != Controller.AutoStage)
                {
                    Controller.AutoStage = ast;
                }

                bool isCoasting = vehicle.Situation == Situation.Maneuvering && Controller.Phase == GravityController.PhaseEnum.Idle
                    || vehicle.Situation == Situation.Freefall && Controller.Phase == GravityController.PhaseEnum.Idle;
                float xl = ImGui.GetCursorPosX();
                float yl = ImGui.GetCursorPosY();
                ImGui.SetCursorPosX(width - 120 - ImGui.GetStyle().ItemInnerSpacing.X*2);
                ImGui.SetCursorPosY(y);
                ImGui.BeginDisabled(isCoasting);
                if (ImGui.Button(vehicle.Situation == Situation.Landed ? "Launch!" : "Abort", new float2(120, yl - y - ImGui.GetStyle().ItemInnerSpacing.Y)))
                {
                    if (vehicle.Situation == Situation.Landed)
                    {
                        Controller.SetVehicle(vehicle);
                        Controller.Launch(vehicle);
                    }
                    else
                    {
                        vehicle.SetEnum(VehicleEngine.MainShutdown);
                        Controller.Phase = GravityController.PhaseEnum.Cleanup;
                        Controller.RunPhaseCleanup(vehicle);
                    }

                }
                ImGui.EndDisabled();
                ImGui.SetCursorPosX(xl);
                ImGui.SetCursorPosY(yl);

                ImGui.Separator();


                double hoa = Controller.GetApoapsisAltitude();
                double hop = Controller.GetPeriapsisAltitude();
                ImGui.TextColored(new float4(1, 0.5f, 0, 1), "Telemetry:");
                ImGui.TextColored(new float4(0.2f, 1, 0.2f, 1), "Phase: " + Controller.Phase.ToString() + ", " + vehicle.Situation.ToString() + ", " + vehicle.FlightComputer.ActiveControlSystem.X.ToString() + ", " + Math.Round(Universe.GetElapsedSeconds() - Controller.LastTransitionTime,1) + "s");
                ImGui.Text("Target time to AP:    " + Controller.TimeToApoapsisTarget.ToString("n1") + "s");
                ImGui.Text("Actual time to AP:    " + Controller.GetApoapsisTime().ToString("n1") + "s (" + (hoa / 1000).ToString("n0") + "km - " + (hop / 1000).ToString("n0") + "km, " + GravityController.RadToDeg(vehicle.Orbit.Inclination).ToString("n1") + "°)");
                ImGui.Separator();
                double vss = vehicle.GetSurfaceSpeed();
                ImGui.Text("Speed (surface):      " + (vss / 1000).ToString("n2") + "km/s");
                ImGui.Text("Altitude:             " + (vehicle.GetRadarAltitude() / 1000).ToString("n1") + "km");
                //ImGui.Text("TWR:                  " + vehicle.NavBallData.ThrustWeightRatio.ToString("n2"));
                
                ImGui.Separator();
                ImGui.Text("Burn dV:              " + Controller.DeltaVUsed.ToString("n0") + "m/s");
                ImGui.Separator();
                if (vehicle.FlightComputer.Burn != null && vehicle.FlightComputer.Burn.BurnDuration > 0.1)
                    ImGui.Text(String.Format("Burn:                 {0}s", vehicle.FlightComputer.Burn.BurnDuration.ToString("n1")));
                else if (Controller.PitchesUp)
                    ImGui.Text("Pitching up:          " + Controller.PitchesUp + ", pitch: " + Program.AttitudePitch.Current);
				//ImGui.Text("Target:               " + vehicle.FlightComputer.AttitudeTrackTarget.ToString() + ", " + vehicle.FlightComputer.AttitudeFrame.ToString());

			}

			ImGui.End();

            flags = ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoBackground | ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.MenuBar;

        }

        [StarMapImmediateLoad]
        public void OnImmediateLoad(Mod mod)
        {
            Console.WriteLine("GravityTurn - On immediate loaded");

        }
        private static bool _autoLoaded;

        [StarMapAllModsLoaded]
        public void OnFullyLoaded()
        {
            Console.WriteLine("GravityTurn - On fully loaded");
            Patcher.Patch();

            new Harmony("gravityturn.autoload").Patch(
                AccessTools.Method(typeof(Program), "OnFrame", new[] { typeof(double), typeof(double) }),
                postfix: new HarmonyMethod(typeof(GravityTurn), nameof(AutoLoad)));
        }

        [StarMapUnload]
        public void Unload()
        {
            Console.WriteLine("GravityTurn - Unload");
            Patcher.Unload();
        }
        private static void AutoLoad()
        {
            if (_autoLoaded) return;
            _autoLoaded = true;
            Program.TerminalInterface.Execute("load Launch");
        }

    }
}
