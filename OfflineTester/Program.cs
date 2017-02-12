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
	static class Program
	{
		private static string pathDirectory = @"C:\Users\Mine's End\AppData\LocalLow\Ludeon Studios\RimWorld by Ludeon Studios\DevOutput";

		static void Main()
		{
			InitGameData();
			Console.WriteLine("-----");
			foreach (var file in Directory.GetFiles(pathDirectory,"*.xml"))
			{
				Console.WriteLine(Path.GetFileNameWithoutExtension(file));
				var pathData = PathDataLog.LoadFromFile(file);

				var map = MapBuilder.MapFromPathData(pathData);
				var start = pathData.start;
				var dest = new LocalTargetInfo(pathData.dest.CenterCell);
				var tp = new TraverseParms() {canBash = pathData.tpCanBash, maxDanger = pathData.tpMaxDanger, mode = pathData.tpMode};
				var peMode = pathData.peMode;

				var pf = new NewPathFinder(map);

				SetPathfinderDelegates(pf, pathData);

				pf.FindPath(start, dest, tp, peMode);
				Console.WriteLine();
			}
			//Console.WriteLine($"pathmax: {true}, weight: {true}, pops: {pf.debug_openCellsPopped}, pathcost: {path.TotalCost}, elapsed: {sw.ElapsedTicks}");
			Console.Read();
		}

		private static void SetPathfinderDelegates(NewPathFinder pf, PathDataLog pathData)
		{
			if (pathData.tpMoveCardinal >= 0)
			{
				NewPathFinder.GetPawnPathCostSettings = (p) => new NewPathFinder.PawnPathCostSettings
				{
					moveTicksCardinal = pathData.tpMoveCardinal,
					moveTicksDiagonal = pathData.tpMoveDiagonal,
					area = pathData.allowedArea,
					avoidGrid = pathData.avoidGrid
				};
			}

            NewPathFinder.GetPathCostForBuilding = (building, parms) =>
			{
				if (building is Building_DoorIndex)
				{
					var fakeEdificeVal = pathData.fakeEdificeGrid[((Building_DoorIndex) building).edificeIndex];
					switch (fakeEdificeVal)
					{
						case PathDataLog.Edifice_Impassible: //shouldn't happen, but might as well check
						case PathDataLog.Edifice_NonTraversableDoor:
							return -1;
						default:
							return fakeEdificeVal;
					}
				}
				if (building is Building_KnownArmedTrap) { return 800; }
				throw new Exception("Unknown Building");
			};
		}

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


		//Mono.Cecil code to prevent UnityEngine calls that will fail
		//static void Run()
		//{
		//	var assemblyDef = AssemblyDefinition.ReadAssembly(@"..\..\Assembly-CSharp.dll");

		//	SetupOfflineTestDll(assemblyDef.MainModule);

		//	assemblyDef.Write("Assembly-csharp-offline.dll");
		//}

		//static void SetupOfflineTestDll(ModuleDefinition module)
		//{
		//  //Redirect log calls to the console
		//	type = module.GetType("Verse.Log");
		//	var logMethod = module.ImportReference(typeof(Console).GetMethod("WriteLine", new[] { typeof(string) }));
		//	foreach (var meth in type.Methods.Where(m => m.Parameters.Count == 1 && m.Parameters[0].Name == "text"))
		//	{
		//		var ilProc = meth.Body.GetILProcessor();
		//		ilProc.Body.Instructions.Clear();
		//		ilProc.Append(ilProc.Create(OpCodes.Ldarg_0));
		//		ilProc.Append(ilProc.Create(OpCodes.Call, logMethod));
		//	}

		//	var type = module.GetType("Verse.ShaderDatabase");
		//	{
		//		var method = type.Methods.First(m => m.Name == "LoadShader");
		//		ReplaceWithReturnNull(method);
		//	}
		//	type = module.GetType("Verse.MaterialPool");
		//	foreach (var meth in type.Methods) { ReplaceWithReturnNull(meth); }

		//	type = module.GetType("Verse.SolidColorMaterials");
		//	foreach (var meth in type.Methods) { ReplaceWithReturnNull(meth); }

		//  //This doesn't directly call any Unity code, but it adds a lot of game state dependencies, some of which do call Unity
		//	type = module.GetType("Verse.MapTemperature");
		//	foreach (var meth in type.Methods.Where(m => m.Name == "CalculateOutdoorTemperatureAtTile"))
		//	{
		//		var ilProc = meth.Body.GetILProcessor();
		//		ilProc.Body.Instructions.Clear();
		//		ilProc.Append(ilProc.Create(OpCodes.Ldc_R4, 0f));
		//		ilProc.Append(ilProc.Create(OpCodes.Ret));
		//	}

		//}

		//static void ReplaceWithReturnNull(MethodDefinition method)
		//{
		//	var ilProc = method.Body.GetILProcessor();
		//	ilProc.Body.Instructions.Clear();
		//	ilProc.Append(ilProc.Create(OpCodes.Ldnull));
		//	ilProc.Append(ilProc.Create(OpCodes.Ret));
		//}
	}
}
