using System.Collections.Generic;
using UnityEngine;
using Waterfall;

namespace WarpThrust
{
    public class WarpEngine
    {
        public string Effect = "";
        public ModuleWaterfallFX Waterfall = null;
        public float MaxThrust = 0;
        public float MinThrottle = 0;
        public Vector3 EngineDir = Vector3.zero;
        public List<int> PropId = new List<int>();
        public List<float> PropFlow = new List<float>();
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class WarpThrust : PartModule
    {
        [KSPField(guiActiveEditor = false, isPersistant = false)]
        double EcRate = 0;

        const string TAG = "[WarpThrust]";
        const string groupName = "WarpThrust";
        const string toggleRot = "Toggle Thrust direction";
        const string toggleUse = "Toggle Engine activation";
        const string toggleRotate = "Toggle automatic rotation";

        bool timeWarp = false;
        bool useRotation = true;
        bool useActive = true;
        bool rotate = true;
        float Throttle = 0f;
        float simThrottle = 0f;
        float PropsFlow = 0f;
        int transforms = 0;

        ModuleEngines Engines;
        ModuleEnginesFX EngineFX;
        ModuleWaterfallFX WaterfallFX;

        List<Propellant> Propellants;
        List<WarpEngine> WarpEngines = new List<WarpEngine>();

        Vector3 TotalDir = Vector3.zero;
        Vector3 WantedRot = Vector3.zero;

        #region Events
        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Thrust mode: Thrusting SAS", active = false, groupName = groupName, groupDisplayName = groupName)]
        protected void UseRot()
        {
            useRotation = true;
            Events["UseSAS"].active = true;
            Events["UseRot"].active = false;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Thrust mode: Thrusting forward", active = true, groupName = groupName, groupDisplayName = groupName)]
        protected void UseSAS()
        {
            useRotation = false;
            Events["UseSAS"].active = false;
            Events["UseRot"].active = true;
        }

        [KSPAction(toggleRot)]
        public void ToggleRot(KSPActionParam param)
        {
            useRotation = !useRotation;
            Events["UseSAS"].active = useRotation;
            Events["UseRot"].active = !useRotation;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Engines: Using all engines", active = false, groupName = groupName, groupDisplayName = groupName)]
        protected void UseActive()
        {
            useActive = true;
            Events["UseThrottle"].active = true;
            Events["UseActive"].active = false;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Engines: Only using an active engine", active = true, groupName = groupName, groupDisplayName = groupName)]
        protected void UseThrottle()
        {
            useActive = false;
            Events["UseThrottle"].active = false;
            Events["UseActive"].active = true;
        }

        [KSPAction(toggleUse)]
        public void ToggleUse(KSPActionParam param)
        {
            useActive = !useActive;
            Events["UseThrottle"].active = useActive;
            Events["UseActive"].active = !useActive;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Rotation: Not Rotating", active = false, groupName = groupName, groupDisplayName = groupName)]
        protected void Rotate()
        {
            rotate = true;
            Events["NotRotate"].active = true;
            Events["Rotate"].active = false;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Rotation: Automatically Rotating", active = true, groupName = groupName, groupDisplayName = groupName)]
        protected void NotRotate()
        {
            rotate = false;
            Events["NotRotate"].active = false;
            Events["Rotate"].active = true;
        }

        [KSPAction(toggleRotate)]
        public void ToggleRotate(KSPActionParam param)
        {
            rotate = !rotate;
            Events["NotRotate"].active = rotate;
            Events["Rotate"].active = !rotate;
        }
        #endregion 


        public ManeuverNode FindactiveManeuver(Vessel vessel)
        {
            List<ManeuverNode> nodes = new List<ManeuverNode>();
            nodes = vessel.patchedConicSolver.maneuverNodes;
            ManeuverNode currentNode = null;
            double currentNodeTime = 0;
            //print(nodes.Count);
            foreach (ManeuverNode node in nodes)
            {
                if (currentNode == null || currentNodeTime > node.UT)
                {
                    currentNode = node;
                    currentNodeTime = node.UT;
                }
            }
            return currentNode;
        }

        public Vector3 GetTargetPositionAtUt(ITargetable Target, double UT)
        {
            Vector3 targetPosition = Target.GetOrbit().getPositionAtUT(UT);
            CelestialBody referenceBody = Target.GetOrbit().referenceBody;
            int i = 0;
            while (i < 10 && referenceBody != null)
            {
                //print(referenceBody.name);
                targetPosition = targetPosition + referenceBody.getPositionAtUT(UT);
                if (referenceBody.GetOrbit() == null)
                {
                    break;
                }
                referenceBody = referenceBody.GetOrbit().referenceBody;
            }
            return targetPosition;
        }

        public Vector3 GetVesselPositionAtUt(Vessel vessel, double UT)
        {
            Vector3 vesselPosition = vessel.GetOrbit().getPositionAtUT(UT);
            CelestialBody referenceBody = vessel.GetOrbit().referenceBody;
            int i = 0;
            while (i < 10 && referenceBody != null)
            {
                //print(referenceBody.name);
                vesselPosition = vesselPosition + referenceBody.getPositionAtUT(UT);
                if (referenceBody.GetOrbit() == null)
                {
                    break;
                }
                referenceBody = referenceBody.GetOrbit().referenceBody;
            }
            return vesselPosition;
        }

        public Vector3 CalcWantedOrbRot(Vessel vessel, double UT, Vector3 WantedRot)
        {
            if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.Prograde)
            {
                return vessel.orbit.Prograde(UT);
            }
            else if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.Retrograde)
            {
                return -vessel.orbit.Prograde(UT);
            }
            else if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.Normal)
            {
                return -vessel.orbit.Normal(UT);
            }
            else if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.Antinormal)
            {
                return vessel.orbit.Normal(UT);
            }
            else if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.RadialIn)
            {
                return vessel.orbit.Radial(UT);
            }
            else if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.RadialOut)
            {
                return -vessel.orbit.Radial(UT);
            }
            else if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.Target)
            {
                Vector3 vesselPosition = GetVesselPositionAtUt(vessel, UT);
                Vector3 targetPosition = GetTargetPositionAtUt(vessel.targetObject, UT);
                return (targetPosition - vesselPosition).normalized;
            }
            else if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.AntiTarget)
            {
                Vector3 vesselPosition = GetVesselPositionAtUt(vessel, UT);
                Vector3 targetPosition = GetTargetPositionAtUt(vessel.targetObject, UT);
                return -(targetPosition - vesselPosition).normalized;
            }
            else if (vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.Maneuver)
            {
                ManeuverNode node = FindactiveManeuver(vessel);
                return (node.nodeRotation * node.DeltaV.normalized).normalized;
            }
            else
            {
                return WantedRot;
            }
        }

        public void Perturb(Orbit orbit, Vector3d deltaVV, double UT) //thanks persistent thrust
        {
            if (deltaVV.magnitude == 0)
                return;

            // Transpose deltaVV Y and Z to match serializedOrbit frame
            Vector3d deltaVVector_orbit = deltaVV.xzy;

            // Position vector
            Vector3d position = orbit.getRelativePositionAtUT(UT);

            // Update with current position and new velocity
            orbit.UpdateFromStateVectors(position, orbit.getOrbitalVelocityAtUT(UT) + deltaVVector_orbit, orbit.referenceBody, UT);
            orbit.Init();
            orbit.UpdateFromUT(UT);
        }

        public void FixedUpdate()
        {
            if (vessel.BestSituation == Vessel.Situations.ORBITING || vessel.BestSituation == Vessel.Situations.ESCAPING || vessel.BestSituation == Vessel.Situations.SUB_ORBITAL)
            {
                if (TimeWarp.CurrentRate != 1 && TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
                {
                    Engines = part.FindModuleImplementing<ModuleEngines>();
                    if (Throttle != 0f)
                    {
                        WantedRot = CalcWantedOrbRot(vessel, Planetarium.GetUniversalTime(), WantedRot);
                        if (rotate)
                        {
                            part.Rigidbody.angularVelocity = Vector3.zero;
                            vessel.transform.Rotate(Quaternion.FromToRotation(vessel.transform.up.normalized, WantedRot).eulerAngles, Space.World); //thanks persistent thrust
                            vessel.SetRotation(vessel.transform.rotation);
                        }
                        vessel.ctrlState.mainThrottle = Throttle;
                        foreach (WarpEngine Engine in WarpEngines)
                        {
                            simThrottle = Engine.MinThrottle + (1 - Engine.MinThrottle) * Throttle;
                            part.Effect(Engine.Effect, simThrottle, -1);
                            foreach (Transform Thrusttransform in Engines.thrustTransforms)
                            {
                                TotalDir -= Thrusttransform.forward;
                                //EnginesDir = Thrusttransform.localRotation;
                                transforms += 1;
                            }
                            Engine.EngineDir = TotalDir / transforms;
                            if (useRotation)
                            {
                                Perturb(vessel.orbit, Engine.EngineDir * (float)(TimeWarp.fixedDeltaTime * simThrottle * Engine.MaxThrust / vessel.totalMass), Planetarium.GetUniversalTime());
                            }
                            else
                            {
                                Perturb(vessel.orbit, WantedRot * (float)(TimeWarp.fixedDeltaTime * simThrottle * Engine.MaxThrust / vessel.totalMass), Planetarium.GetUniversalTime());
                            }
                            TotalDir = Vector3.zero;
                            transforms = 0;
                            if (EcRate != 0)
                            {
                                double ElectricCharge = part.RequestResource("ElectricCharge", EcRate * TimeWarp.fixedDeltaTime * simThrottle);
                                if (ElectricCharge == 0)
                                {
                                    ScreenMessages.PostScreenMessage(TAG + " Too little electric charge remaining!\nShutting down the engines");
                                    Throttle = 0f;
                                    vessel.ctrlState.mainThrottle = Throttle;
                                    simThrottle = Engine.MinThrottle + (1 - Engine.MinThrottle) * Throttle;
                                }
                            }
                            for (int i = 0; i < Engine.PropId.Count; i++)
                            {
                                double Fuel = part.RequestResource(Engine.PropId[i], (double)(Engine.PropFlow[i] * TimeWarp.fixedDeltaTime * simThrottle));
                                if (Fuel == 0)
                                {
                                    ScreenMessages.PostScreenMessage(TAG + " Too little fuel remaining!\nShutting down the engines");
                                    Throttle = 0f;
                                    vessel.ctrlState.mainThrottle = Throttle;
                                    simThrottle = Engine.MinThrottle + (1 - Engine.MinThrottle) * Throttle;
                                }
                            }
                            if (vessel.BestSituation == Vessel.Situations.SUB_ORBITAL)
                            {
                                ScreenMessages.PostScreenMessage(TAG + "Orbit too low!\nShutting down the engines");
                                Throttle = 0f;
                                vessel.ctrlState.mainThrottle = Throttle;
                                simThrottle = Engine.MinThrottle + (1 - Engine.MinThrottle) * Throttle;
                            }
                            if (Engine.Waterfall != null)
                            {
                                foreach (WaterfallController Controller in Engine.Waterfall.Controllers)
                                {
                                    if (Controller.name == "throttle")
                                    {
                                        Controller.overridden = true;
                                        Controller.overrideValue = simThrottle;
                                    }
                                }
                            }
                            timeWarp = true;
                        }
                    }
                }
                else
                {
                    WantedRot = vessel.Autopilot.SAS.targetOrientation;
                    if (useActive)
                    {
                        Throttle = Engines.requestedThrottle; //vessel.ctrlState.mainThrottle;
                    }
                    else
                    {
                        Throttle = vessel.ctrlState.mainThrottle;
                    }
                    if (timeWarp)
                    {
                        foreach (WarpEngine Engine in WarpEngines)
                        {
                            if (Engine.Waterfall != null)
                            {
                                foreach (WaterfallController Controller in Engine.Waterfall.Controllers)
                                {
                                    if (Controller.name == "throttle")
                                    {
                                        Controller.overridden = false;
                                        Controller.overrideValue = Engine.MinThrottle + (1 - Engine.MinThrottle) * Throttle;
                                    }
                                }
                            }
                        }
                    }
                    timeWarp = false;
                }
            }
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);

            Engines = part.FindModuleImplementing<ModuleEngines>();
            EngineFX = part.FindModuleImplementing<ModuleEnginesFX>();
            WaterfallFX = part.FindModuleImplementing<ModuleWaterfallFX>();

            WarpEngines.Add(new WarpEngine());
            WarpEngines[WarpEngines.Count - 1].Waterfall = WaterfallFX;
            WarpEngines[WarpEngines.Count - 1].Effect = EngineFX.powerEffectName;
            WarpEngines[WarpEngines.Count - 1].MaxThrust = Engines.maxThrust;
            WarpEngines[WarpEngines.Count - 1].MinThrottle = Engines.throttleMin;
            Propellants = Engines.propellants;
            PropsFlow = Engines.maxFuelFlow / Engines.mixtureDensity;
            TotalDir = Vector3.zero;
            transforms = 0;
            foreach (Propellant Propellant in Propellants)
            {
                WarpEngines[WarpEngines.Count - 1].PropId.Add(Propellant.id);
                WarpEngines[WarpEngines.Count - 1].PropFlow.Add(Propellant.ratio * PropsFlow);
            }
        }
    }
}
