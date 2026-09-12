namespace AdvancedFlightComputer.Features.AutoStage;

// Type marker for the AUTOSTAGE gauge button. GaugeButtonFlightComputer resolves it by Type.Name
// with the first hit, so the name is distinct from the standalone AutoStage mod's marker and the
// two buttons cannot bind to each other's type. The value is never read.
public enum AfcAutoStageToggle { Enabled }
