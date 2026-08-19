namespace Scvx;

/// <summary>
/// A sparse matrix in the column-compressed storage ECOS expects, built once and
/// then refilled in place.
///
/// This is the piece that makes warm starting worthwhile. Across SCvx iterations
/// only the NUMBERS change — the linearisation blocks, the reference trajectory,
/// the trust radius — while which entries are non-zero is fixed by the problem
/// structure. Rebuilding the pattern every iteration (sorting triplets, grouping
/// duplicates, reallocating) is what makes the Python reference spend ~80% of its
/// time in CVXPY canonicalisation rather than in the solver.
///
/// So: the first assembly pass records the pattern and the values; every later
/// pass replays the same <see cref="Add"/> calls in the same order and only
/// writes values. The CCS arrays are allocated once and never move, which is also
/// what ECOS_updateData needs — it keeps the symbolic factorisation and only
/// refreshes the KKT entries.
///
/// The contract that makes this safe: <b>the assembly code must call Add in an
/// identical order every pass.</b> Never skip an entry because it happens to be
/// zero this time round — explicit zeros are harmless, a shifted order is not.
/// <see cref="Refill"/> checks the count, so a divergence fails loudly instead of
/// silently writing values into the wrong coefficients.
/// </summary>
public sealed class CcsAssembler
{
    public int Rows { get; }
    public int Cols { get; }

    private readonly List<int> _row = [];
    private readonly List<int> _col = [];
    private readonly List<double> _scratch = [];

    private bool _frozen;
    private int _cursor;

    // CCS arrays, allocated at Freeze() and then stable for the object's life.
    private double[] _pr = [];
    private int[] _jc = [];
    private int[] _ir = [];
    // entry ordinal (order of Add calls) -> index into _pr
    private int[] _slot = [];

    public CcsAssembler(int rows, int cols)
    {
        Rows = rows;
        Cols = cols;
    }

    public double[] Values => _pr;
    public int[] ColumnPointers => _jc;
    public int[] RowIndices => _ir;
    public int NonZeros => _pr.Length;

    /// <summary>Number of Add calls the pattern expects per pass.</summary>
    public int EntryCount => _row.Count;

    public void Add(int row, int col, double value)
    {
        if (!_frozen)
        {
            if ((uint)row >= (uint)Rows || (uint)col >= (uint)Cols)
                throw new ArgumentOutOfRangeException(
                    nameof(row), $"({row},{col}) outside {Rows}x{Cols}");
            _row.Add(row);
            _col.Add(col);
            _scratch.Add(value);
            return;
        }

        if (_cursor >= _slot.Length)
            throw new InvalidOperationException(
                $"assembly produced more than the {_slot.Length} entries in the frozen " +
                "pattern — the Add sequence must be identical every pass");
        // Accumulate, not assign: BeginRefill zeroes the values, and duplicate
        // (row, col) entries share a slot, so they must sum the same way Freeze
        // summed them on the first pass.
        _pr[_slot[_cursor++]] += value;
    }

    /// <summary>
    /// Finish the first pass: sort into CCS order and build the entry-to-slot map.
    /// Duplicate (row, col) entries are summed, exactly as the pattern-free
    /// builder did — but note that duplicates make refills ambiguous, so the
    /// assembly deliberately emits each coefficient combined and once.
    /// </summary>
    public void Freeze()
    {
        if (_frozen) throw new InvalidOperationException("already frozen");

        int nnz = _row.Count;
        // Order entries by (column, row) — CCS wants rows ascending within a column.
        var order = new int[nnz];
        for (int i = 0; i < nnz; i++) order[i] = i;
        Array.Sort(order, (a, b) =>
        {
            int c = _col[a].CompareTo(_col[b]);
            return c != 0 ? c : _row[a].CompareTo(_row[b]);
        });

        var prList = new List<double>(nnz);
        var irList = new List<int>(nnz);
        _jc = new int[Cols + 1];
        _slot = new int[nnz];

        int col = 0;
        _jc[0] = 0;
        for (int k = 0; k < nnz; k++)
        {
            int e = order[k];
            while (col < _col[e]) _jc[++col] = prList.Count;

            // Same position as the previous entry? Fold into it (summed), and map
            // this ordinal to the same slot so a refill accumulates identically.
            bool same = k > 0 && _col[order[k - 1]] == _col[e] && _row[order[k - 1]] == _row[e];
            if (same)
            {
                _slot[e] = prList.Count - 1;
                prList[^1] += _scratch[e];
            }
            else
            {
                _slot[e] = prList.Count;
                irList.Add(_row[e]);
                prList.Add(_scratch[e]);
            }
        }
        while (col < Cols) _jc[++col] = prList.Count;

        _pr = [.. prList];
        _ir = [.. irList];
        _frozen = true;
        _cursor = 0;
        _scratch.Clear();
        _scratch.TrimExcess();
    }

    /// <summary>Begin a refill pass. Values are zeroed so duplicate positions accumulate.</summary>
    public void BeginRefill()
    {
        if (!_frozen) throw new InvalidOperationException("freeze the pattern first");
        _cursor = 0;
        Array.Clear(_pr);
    }

    /// <summary>Assert the refill pass matched the frozen pattern exactly.</summary>
    public void EndRefill()
    {
        if (_cursor != _slot.Length)
            throw new InvalidOperationException(
                $"assembly produced {_cursor} entries but the frozen pattern has " +
                $"{_slot.Length} — the Add sequence must be identical every pass");
    }
}
