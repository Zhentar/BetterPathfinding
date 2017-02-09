using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BetterPathfinding;
using RimWorld;
using Verse;
using Verse.AI;

namespace OfflineTester
{
	static class MapBuilder
	{

		public static Map MapFromPathData(PathDataDumper data)
		{
			var map = new Map();
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

			map.pathGrid.pathGrid = data.pathGrid.ToArray();

			var edificeAry = map.edificeGrid.InnerArray;

			for (int i = 0; i < data.fakeEdificeGrid.CellsCount; i++)
			{
				switch (data.fakeEdificeGrid[i])
				{
					case PathDataDumper.Edifice_Impassible:
						edificeAry[i] = Impassible;
						break;
				}
			}

			return map;
		}

		private static Building Impassible = InitImpassibleBuilding();

		private static Building InitImpassibleBuilding()
		{
			try
			{
				var building = new Building {def = new ThingDef {regionBarrier = true}};
				return building;
			}
			catch { }
			return new Building { def = new ThingDef { regionBarrier = true } };
		}
	}
}
