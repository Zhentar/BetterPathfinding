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
		    var sw = new Stopwatch();
            foreach (var file in Directory.GetFiles(pathDirectory,"*.xml"))
		    //var file = Path.Combine(pathDirectory, "Darcie - 1215368.xml");
            {
                GC.Collect();
				Console.WriteLine(Path.GetFileNameWithoutExtension(file));

                var pathData = PathDataLog.LoadFromFile(file);

                var pf = new NewPathFinder(MapBuilder.MapFromPathData(pathData));
                var dest = new LocalTargetInfo(pathData.dest.CenterCell);
				var tp = new TraverseParms() {canBash = pathData.tpCanBash, maxDanger = pathData.tpMaxDanger, mode = pathData.tpMode};

				SetPathfinderDelegates(pf, pathData);

                sw.Start();
                //for (int i = 0; i < 4; i++)
                {
                    var path = pf.FindPath(pathData.start, dest, tp, pathData.peMode);
                    path.Dispose();
                }
                sw.Stop();
				Console.WriteLine();
			}
			//Console.WriteLine($"pathmax: {true}, weight: {true}, pops: {pf.debug_openCellsPopped}, pathcost: {path.TotalCost}, elapsed: {sw.ElapsedTicks}");
		    Console.WriteLine($"\nAll Done!\nElapsed pathfinding time: {sw.ElapsedMilliseconds}ms");
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
		//	var type = module.GetType("Verse.ShaderDatabase");
		//	{
		//		var method = type.Methods.First(m => m.Name == "LoadShader");
		//		ReplaceWithReturnNull(method);
		//	}
		//	type = module.GetType("Verse.Log");
		//	var logMethod = module.ImportReference(typeof(Console).GetMethod("WriteLine", new[] { typeof(string) }));
		//	foreach (var meth in type.Methods.Where(m => m.Parameters.Count == 1 && m.Parameters[0].Name == "text"))
		//	{
		//		var ilProc = meth.Body.GetILProcessor();
		//		ilProc.Body.Instructions.Clear();
		//		ilProc.Emit(OpCodes.Ldarg_0);
		//		ilProc.Emit(OpCodes.Call, logMethod);
		//		ilProc.Emit(OpCodes.Ret);
		//	}
		//	type = module.GetType("Verse.MaterialPool");
		//	foreach (var meth in type.Methods) { ReplaceWithReturnNull(meth); }

		//	type = module.GetType("Verse.SolidColorMaterials");
		//	foreach (var meth in type.Methods) { ReplaceWithReturnNull(meth); }

		//	type = module.GetType("Verse.MapTemperature");
		//	foreach (var meth in type.Methods.Where(m => m.Name == "CalculateOutdoorTemperatureAtTile"))
		//	{
		//		var ilProc = meth.Body.GetILProcessor();
		//		ilProc.Body.Instructions.Clear();
		//		ilProc.Emit(OpCodes.Ldc_R4, 0f);
		//		ilProc.Emit(OpCodes.Ret);
		//	}

		//	//This works fine with .NET 3.5 (it errors gracefully), but with newer runtimes it throws an invalid program exception
		//	//So we'll null it out so we can use the latest & greatest profiling & debugging features
		//	type = module.GetType("Verse.ContentFinder`1");
		//	ReplaceWithGenericReturnNull(type.Methods.First(m => m.Name == "Get"));

		//}

		//static void ReplaceWithReturnNull(MethodDefinition method)
		//{
		//	var ilProc = method.Body.GetILProcessor();
		//	ilProc.Body.Instructions.Clear();
		//	ilProc.Body.ExceptionHandlers.Clear();
		//	ilProc.Emit(OpCodes.Ldnull);
		//	ilProc.Emit(OpCodes.Ret);
		//}

		//static void ReplaceWithGenericReturnNull(MethodDefinition method)
		//{
		//	var ilProc = method.Body.GetILProcessor();
		//	ilProc.Body.Instructions.Clear();
		//	ilProc.Body.ExceptionHandlers.Clear();
		//	ilProc.Emit(OpCodes.Ldloca_S, method.Body.Variables[0]);
		//	ilProc.Emit(OpCodes.Initobj, method.Body.Variables[0].VariableType);
		//	ilProc.Emit(OpCodes.Ldloc_0);
		//	ilProc.Emit(OpCodes.Ret);
		//}
	}
}
