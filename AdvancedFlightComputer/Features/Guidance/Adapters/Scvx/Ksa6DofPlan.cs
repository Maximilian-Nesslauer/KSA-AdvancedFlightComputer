/// <summary>
/// A finished trajectory, published as one immutable object.
///
/// This is the ONLY thing the sim thread reads from a solve. Command needs four
/// things that are written together — the control history, the burn time, the time the
/// plan was anchored, and the node count — and reading them as four separate fields is
/// a torn read waiting to happen: new controls paired with an old anchor time means
/// the vehicle is commanded from the wrong point of the right trajectory, which looks
/// exactly like the guidance ignoring its own plan.
///
/// Publishing is a single reference assignment, so a reader either sees the whole
/// previous plan or the whole new one and never a mixture. No lock is involved, and
/// none is needed: the arrays are freshly allocated by each solve and never written
/// again once published, so they are immutable in practice even though double[] cannot
/// say so in the type system. Nothing may mutate them in place — that is the one rule
/// this type depends on.
/// </summary>
/// <param name="X">Node states, Nodes * Dynamics6Dof.NX. Never mutated after publish.</param>
/// <param name="U">Node controls, Nodes * Dynamics6Dof.NU. Never mutated after publish.</param>
/// <param name="Sigma">Burn time this plan covers, seconds.</param>
/// <param name="SolveTime">Sim time at which node 0 was the vehicle.</param>
/// <param name="Nodes">Node count, so the reader does not have to infer it from a length.</param>
public sealed record Ksa6DofPlan(double[] X, double[] U, double Sigma, double SolveTime, int Nodes);
