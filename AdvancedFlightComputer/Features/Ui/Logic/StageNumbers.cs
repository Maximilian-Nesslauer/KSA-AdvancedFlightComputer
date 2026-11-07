using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct StageHueRange(int First, int Count, int Second)
{
    public static readonly StageHueRange None = new(-1, 0, -1);

    public bool Any => Count > 0;

    public bool TwoHues => Second != First;
}

internal static class StageNumbers
{
    public static int[] Of(ReadOnlySpan<int> stack, ReadOnlySpan<int> left)
    {
        var numbers = new int[left.Length];
        if (Runs(left) > Runs(stack))
        {
            for (int i = 1; i < left.Length; i++)
                numbers[i] = left[i] == left[i - 1] ? numbers[i - 1] : numbers[i - 1] + 1;
            return numbers;
        }

        int j = stack.Length - 1;
        for (int i = left.Length - 1; i >= 0;)
        {
            RunOf(left, i, out int leftFirst, out _);
            RunOf(stack, j, out int stackFirst, out _);
            for (int k = leftFirst; k <= i; k++)
                numbers[k] = Math.Min(stackFirst + (k - leftFirst), j);
            i = leftFirst - 1;
            j = stackFirst - 1;
        }
        return numbers;
    }

    public static bool Fits(ReadOnlySpan<int> stack, ReadOnlySpan<int> left) => Runs(left) <= Runs(stack);

    public static string Tag(ReadOnlySpan<int> stack, int index)
    {
        if (index < 0 || index >= stack.Length || stack[index] < 0)
            return "S" + (index + 1).ToString(Fmt.Inv);
        int seq = stack[index];
        RunOf(stack, index, out int first, out int last);
        string name = "Seq " + seq.ToString(Fmt.Inv);
        return last > first ? name + (char)('a' + Math.Min(index - first, 25)) : name;
    }

    public static bool KeepsStack(bool hasStack, bool fits, bool flies, bool flew, bool hasPlan, bool samePlan, bool hadPlan)
    {
        if (!hasStack || !fits)
            return false;
        if (flies)
            return true;
        if (hasPlan)
            return samePlan;
        return !hadPlan && flew;
    }

    public static StageHueRange HuesOf(ReadOnlySpan<int> left, ReadOnlySpan<int> numbers, ReadOnlySpan<int> engines, int sequence)
    {
        int after = 0;
        for (int i = 0; i < left.Length && i < numbers.Length; i++)
        {
            if (left[i] == sequence)
            {
                int count = 1;
                while (i + count < left.Length && i + count < numbers.Length && left[i + count] == sequence)
                    count++;
                return new StageHueRange(numbers[i], count, numbers[i + Math.Min(count - 1, 1)]);
            }
            after = Math.Max(after, numbers[i] + 1);
        }
        int place = engines.IndexOf(sequence);
        if (place < 0)
            return StageHueRange.None;
        int hue = after;
        for (int e = 0; e < place; e++)
        {
            if (!left.Contains(engines[e]))
                hue++;
        }
        return new StageHueRange(hue, 1, hue);
    }

    public static string KeyOf(ReadOnlySpan<int> stack, ReadOnlySpan<int> left, ReadOnlySpan<int> numbers, int stage)
    {
        string plan = Tag(stack, stage);
        if (stage < 0 || stage >= stack.Length || stack[stage] < 0 || left.Length == 0 || numbers.Length == 0)
            return plan;
        int seq = stack[stage];
        RunOf(stack, stage, out int first, out int last);
        bool stays = false;
        foreach (int number in numbers)
            stays |= number >= first && number <= last;
        if (!stays)
            return DroppedWord;
        bool burns = numbers[0] >= first && numbers[0] <= last;
        return burns && left[0] != seq ? plan + ", now " + ShortTag(Tag(left, 0)) : plan;
    }

    public const string DroppedWord = "Dropped";

    public static string ShortTag(string tag) => tag.StartsWith("Seq ", StringComparison.Ordinal) ? tag.Substring(4) : tag;

    private static void RunOf(ReadOnlySpan<int> list, int index, out int first, out int last)
    {
        first = last = index;
        while (first > 0 && list[first - 1] == list[index])
            first--;
        while (last < list.Length - 1 && list[last + 1] == list[index])
            last++;
    }

    private static int Runs(ReadOnlySpan<int> list)
    {
        int runs = list.Length > 0 ? 1 : 0;
        for (int i = 1; i < list.Length; i++)
        {
            if (list[i] != list[i - 1])
                runs++;
        }
        return runs;
    }
}
