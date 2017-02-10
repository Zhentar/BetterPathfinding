using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
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
		private static string pathDirectory = @"C:\Users\Mine's End\AppData\LocalLow\Ludeon Studios\RimWorld by Ludeon Studios\DevOutput";

		static void Main(string[] args)
		{
			InitGameData();
			Console.WriteLine("-----");
			foreach (var file in Directory.GetFiles(pathDirectory))
			{
				var pathData = PathDataDumper.LoadFromFile(file);

				var map = MapBuilder.MapFromPathData(pathData);
				var start = pathData.start;
				var dest = new LocalTargetInfo(pathData.dest.CenterCell);
				var tp = new TraverseParms() {canBash = pathData.tpCanBash, maxDanger = pathData.tpMaxDanger, mode = pathData.tpMode};
				var peMode = pathData.peMode;

				var pf = new NewPathFinder(map);

				pf.GetPawnPathCostSettings = (p) => new NewPathFinder.PawnPathCostSettings
				{
					moveTicksCardinal = pathData.tpMoveCardinal,
					moveTicksDiagonal = pathData.tpMoveDiagonal,
					area = pathData.allowedArea,
					avoidGrid = pathData.avoidGrid
				};

				pf.GetPathCostForBuilding = (building, parms) =>
				{
					var fakeEdificeVal = pathData.fakeEdificeGrid[((Building_DoorIndex) building).edificeIndex];
					switch (fakeEdificeVal)
					{
						case PathDataDumper.Edifice_KnownArmedTrap:
							return 800;
						case PathDataDumper.Edifice_Impassible: //shouldn't happen, but might as well check
						case PathDataDumper.Edifice_NonTraversableDoor:
							return -1;
						default:
							return fakeEdificeVal;
					}
				};

				pf.FindPath(start, dest, tp, peMode);
				Console.WriteLine();
			}
			//Console.WriteLine($"pathmax: {true}, weight: {true}, pops: {pf.debug_openCellsPopped}, pathcost: {path.TotalCost}, elapsed: {sw.ElapsedTicks}");
			Console.Read();
		}

		//public Func<Pawn, NewPathFinder.PawnPathCostSettings> GetPawnPathCostSettings = GetPawnPathCostSettingsDefault;


		//public Func<Building, TraverseParms, int> GetPathCostForBuilding = GetPathCostForBuildingDefault;

		private static void InitGameData()
		{
			ThingCategoryDefOf.Apparel = new ThingCategoryDef(); //OutfitDatabase needs this
			SpecialThingFilterDefOf.AllowNonDeadmansApparel = new SpecialThingFilterDef();
			var drugDef = new ThingDef { category = ThingCategory.Item, ingestible = new IngestibleProperties { drugCategory = DrugCategory.Social } };
			DefDatabase<ThingDef>.AllDefsListForReading.Add(drugDef);
			ThingDefOf.Beer = drugDef;
			ThingDefOf.SmokeleafJoint = drugDef;
			Current.Game = new Game();
			Current.Game.World = new RimWorld.Planet.World();
			Current.Game.World.uniqueIDsManager = new UniqueIDsManager();
			//Current.Game.World.grid = new RimWorld.Planet.WorldGrid();
			//Current.Game.World.grid.tiles.Add(new RimWorld.Planet.Tile());
			var roomDef = new RoomStatDef();
			DefDatabase<RoomStatDef>.AllDefsListForReading.Add(roomDef); //For RegionMaker
		}
	}
}
