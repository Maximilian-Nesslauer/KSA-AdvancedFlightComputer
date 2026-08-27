namespace Gfold;

// Builds a sparse matrix in the column-compressed storage (CCS) format ECOS
// expects, from arbitrary-order (row, col, value) triplets. Duplicate entries
// at the same position are summed; explicit zeros are kept (harmless).
public sealed class SparseCcs
{
    public int Rows { get; }
    public int Cols { get; }

    private readonly List<(int Row, double Value)>[] _columns;

    public SparseCcs(int rows, int cols)
    {
        Rows = rows;
        Cols = cols;
        _columns = new List<(int, double)>[cols];
        for (int j = 0; j < cols; j++)
            _columns[j] = new List<(int, double)>();
    }

    public void Add(int row, int col, double value)
    {
        if ((uint)row >= (uint)Rows || (uint)col >= (uint)Cols)
            throw new ArgumentOutOfRangeException($"({row},{col}) outside {Rows}x{Cols}");
        _columns[col].Add((row, value));
    }

    /// <summary>
    /// Stack two matrices vertically: [top; bottom], with bottom's rows shifted down
    /// by top.Rows. Both must have the same column count.
    ///
    /// This is how the split ECOS form becomes SCS's single constraint matrix — the
    /// equality block on top, the cone block beneath — and it is the one step of the
    /// conversion that is easy to get quietly wrong, because CCS is COLUMN-major: a
    /// vertical stack is not a concatenation of the two arrays but an interleave
    /// within every column. Working in triplets sidesteps that entirely; Build()
    /// sorts each column afterwards regardless.
    /// </summary>
    public static SparseCcs VStack(SparseCcs top, SparseCcs bottom)
    {
        if (top.Cols != bottom.Cols)
            throw new ArgumentException($"column counts differ: {top.Cols} vs {bottom.Cols}");

        var stacked = new SparseCcs(top.Rows + bottom.Rows, top.Cols);
        for (int j = 0; j < top.Cols; j++)
        {
            foreach ((int row, double value) in top._columns[j])
                stacked._columns[j].Add((row, value));
            foreach ((int row, double value) in bottom._columns[j])
                stacked._columns[j].Add((row + top.Rows, value));
        }
        return stacked;
    }

    /// <summary>Nonzeros as (row, col, value), for tests and dense round-trips.</summary>
    public IEnumerable<(int Row, int Col, double Value)> Triplets()
    {
        for (int j = 0; j < Cols; j++)
            foreach ((int row, double value) in _columns[j])
                yield return (row, j, value);
    }

    // (values, column pointers of length Cols+1, row indices), rows sorted
    // ascending within each column as both ECOS and SCS require.
    public (double[] Pr, int[] Jc, int[] Ir) Build()
    {
        var pr = new List<double>();
        var ir = new List<int>();
        var jc = new int[Cols + 1];
        for (int j = 0; j < Cols; j++)
        {
            jc[j] = pr.Count;
            foreach (var group in _columns[j]
                         .GroupBy(e => e.Row)
                         .OrderBy(g => g.Key))
            {
                ir.Add(group.Key);
                pr.Add(group.Sum(e => e.Value));
            }
        }
        jc[Cols] = pr.Count;
        return (pr.ToArray(), jc, ir.ToArray());
    }
}
