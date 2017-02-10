using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using BetterPathfinding;
using RimWorld;
using Verse;
using Verse.AI;

namespace OfflineTester
{
	static class MapBuilder
	{
		private static bool isFirstRun = true;

		public static Map MapFromPathData(PathDataDumper data)
		{
			var map = new Map();
			Find.Maps.Clear();
			Find.Maps.Add(map);
			map.info.Size = data.mapSize;

			map.cellIndices = new CellIndices(map);
			map.temperatureCache = new TemperatureCache(map);
			map.regionLinkDatabase = new RegionLinkDatabase();
			map.regionGrid = new RegionGrid(map);
			map.regionMaker = new RegionMaker(map);
			map.regionAndRoomUpdater = new RegionAndRoomUpdater(map);
			map.pathGrid = new PathGrid(map);
			map.regionDirtyer = new RegionDirtyer(map);
			map.edificeGrid = new EdificeGrid(map);
			map.pawnPathPool = new PawnPathPool(map);
			map.thingGrid = new ThingGrid(map);
			map.mapTemperature = new MapTemperature(map);
			map.mapConditionManager = new MapConditionManager(map);
			map.autoBuildRoofAreaSetter = new AutoBuildRoofAreaSetter(map);
			map.roofGrid = new RoofGrid(map);
			map.mapPawns = new MapPawns(map);
			map.reachability = new Reachability(map);
			map.areaManager = new AreaManager(map);

			map.pathGrid.pathGrid = data.pathGrid.ToArray();

			var edificeAry = map.edificeGrid.InnerArray;

			for (int i = 0; i < data.fakeEdificeGrid.CellsCount; i++)
			{
				switch (data.fakeEdificeGrid[i])
				{
					case PathDataDumper.Edifice_Impassible:
						edificeAry[i] = SetupNewBuilding(new Building(), i);
						break;
					case PathDataDumper.Edifice_None:
						continue;
					//TODO: handle traps
					default:
						edificeAry[i] = SetupNewBuilding(new Building_DoorIndex(i), i);
						map.thingGrid.Register(edificeAry[i]);
						break;	
				}
			}

			//Do this here for more consistent profiling times
			map.regionAndRoomUpdater.RebuildDirtyRegionsAndRooms();

			return map;
		}

		private static Building SetupNewBuilding(Building building, int index)
		{
			building.def = new ThingDef { regionBarrier = true };
			building.Position = Find.Maps[0].cellIndices.IndexToCell(index);
			Building_DoorIndex.mapIndexOrStateInfo.SetValue(building, (sbyte)0);
			building.Map.thingGrid.Register(building);
			return building;
		}

		private static Building Impassible = new Building { def = new ThingDef { regionBarrier = true } };
	}

	public class Building_DoorIndex : Building_Door
	{

		private static readonly Func<Thing, sbyte> mapIndexOrStateGet = Utils.GetFieldAccessor<Thing, sbyte>("mapIndexOrState");

		public static readonly FieldInfo mapIndexOrStateInfo = typeof(Thing).GetField("mapIndexOrState", BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.GetField);

		private sbyte mapIndexOrState { get { return mapIndexOrStateGet(this); } set { mapIndexOrStateInfo.SetValue(this, value); } }


		public readonly int edificeIndex;

		public Building_DoorIndex(int index)
		{
			edificeIndex = index;
		}
	}
}
