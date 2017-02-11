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
		public static Map MapFromPathData(PathDataLog data)
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
				Building building = null;
				switch (data.fakeEdificeGrid[i])
				{
					case PathDataLog.Edifice_None:
						continue;
					case PathDataLog.Edifice_Impassible:
						building = new Building();
						break;
					case PathDataLog.Edifice_KnownArmedTrap:
						building = new Building_KnownArmedTrap();
						break;
					default:
						building = new Building_DoorIndex(i);
						break;
				}
				edificeAry[i] = SetupNewBuilding(building, i);
				map.thingGrid.Register(edificeAry[i]);
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
	}

	public class Building_KnownArmedTrap : Building
	{ }

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
