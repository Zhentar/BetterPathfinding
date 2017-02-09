using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using BetterPathfinding;
using RimWorld;
using RimWorld.Planet;
using Verse;

namespace OfflineTester
{
	class Program
	{
		private static string filename = @"C:\Users\Mine's End\AppData\LocalLow\Ludeon Studios\RimWorld by Ludeon Studios\DevOutput\Darcie - 1215368 2.xml";

		static void Main(string[] args)
		{
			var pathData = PathDataDumper.LoadFromFile(filename);

			ThingCategoryDefOf.Apparel = new ThingCategoryDef(); //OutfitDatabase needs this
			SpecialThingFilterDefOf.AllowNonDeadmansApparel = new SpecialThingFilterDef();
			var drugDef = new ThingDef { category = ThingCategory.Item, ingestible = new IngestibleProperties {drugCategory = DrugCategory.Social}};
			DefDatabase<ThingDef>.AllDefsListForReading.Add(drugDef);
			ThingDefOf.Beer = drugDef;
			ThingDefOf.SmokeleafJoint = drugDef;
			Current.Game = new Game();
			//Current.Game.World = new RimWorld.Planet.World();
			//Current.Game.World.grid = new RimWorld.Planet.WorldGrid();
			//Current.Game.World.grid.tiles.Add(new RimWorld.Planet.Tile());
			var roomDef = new RoomStatDef();
			DefDatabase<RoomStatDef>.AllDefsListForReading.Add(roomDef); //For RegionMaker
			var map = MapBuilder.MapFromPathData(pathData);
			var start = pathData.start;
			var dest = new LocalTargetInfo(pathData.dest.CenterCell);
			var tp = new TraverseParms() {canBash = pathData.tpCanBash, maxDanger = pathData.tpMaxDanger, mode = pathData.tpMode};
			var peMode = pathData.peMode;

			var pf = new NewPathFinder(map);

			var sw = Stopwatch.StartNew();
			var path = pf.FindPath(start, dest, tp, peMode);
			sw.Stop();

			//Console.WriteLine($"pathmax: {true}, weight: {true}, pops: {pf.debug_openCellsPopped}, pathcost: {path.TotalCost}, elapsed: {sw.ElapsedTicks}");
			Console.Read();
		}
	}
}
