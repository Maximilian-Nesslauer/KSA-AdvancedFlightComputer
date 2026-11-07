using System;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.Ui.Canvas;

/// <summary>A reused buffer of screen points for one draw call at a time, so a frame allocates nothing once the buffer has grown to the longest path.</summary>
internal sealed class PointBuffer
{
    private float2[] _pts = new float2[512];

    /// <summary>The buffer, at least <paramref name="n"/> long. The next call may hand out the same array, so a caller is done with it before it asks again.</summary>
    public float2[] Get(int n)
    {
        if (_pts.Length < n)
            _pts = new float2[Math.Max(n, _pts.Length * 2)];
        return _pts;
    }
}
