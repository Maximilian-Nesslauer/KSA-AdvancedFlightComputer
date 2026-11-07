namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// Who flies what a page or a verb commits: nobody, the player, AFC, the stock flight computer, AFC
/// that plans a stock node which stock AUTO or the player then flies, or nobody because the page holds
/// settings only. <see cref="StockThenAfc"/> is a deorbit: stock AUTO flies the burn, then AFC lands.
/// </summary>
internal enum Owner { None, Player, Afc, Stock, AfcPlansStockFlies, Settings, StockThenAfc }
