using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Verse;

namespace BetterPathfinding
{
	class RegionPathCostHeuristic
	{
	
		private readonly IntVec3 startCell;
		private IntVec3 targetCell;
		private readonly int moveTicksCardinal;
		private readonly int moveTicksDiagonal;
		private readonly Map map;
		
		private readonly IEnumerable<Region> rootRegions;

		private RegionLinkDijkstra distanceBuilder;

		private int lastRegionId = -1;
		private RegionLink bestLink;
		private int bestLinkCost;
		private RegionLink secondBestLink;
		private int secondBestLinkCost;
		private int lastRegionTilePathCost;
        private readonly TraverseParms traverseParms;
		private readonly NewPathFinder.PawnPathCostSettings pathCosts;

		private static readonly Func<RegionGrid, Region[]> regionGridGet = Utils.GetFieldAccessor<RegionGrid, Region[]>("regionGrid");

		private static readonly FieldInfo regionGridInfo = typeof(RegionGrid).GetField("regionGrid", BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.GetField);

		private Region[] regionGrid { get { return regionGridGet(this.map.regionGrid); } set { regionGridInfo.SetValue(this, value); } }


		public RegionPathCostHeuristic(Map map, IntVec3 start, CellRect end, IEnumerable<Region> destRegions, TraverseParms parms, NewPathFinder.PawnPathCostSettings pathCosts)
		{
			this.map = map;
			startCell = start;
			targetCell = end.CenterCell;
			moveTicksCardinal = pathCosts.moveTicksCardinal;
			moveTicksDiagonal = pathCosts.moveTicksDiagonal;
			this.pathCosts = pathCosts;

			rootRegions = new HashSet<Region>(destRegions);
            traverseParms = parms;
		}

		public int GetPathCostToRegion(int cellIndex)
		{
			var region = regionGrid[cellIndex];
			var cell = this.map.cellIndices.IndexToCell(cellIndex);

			if (rootRegions.Contains(region))
			{
				var dx = Math.Abs(cell.x - targetCell.x);
				var dz = Math.Abs(cell.z - targetCell.z);
				return OctileDistance(dx, dz);
			}

			if (distanceBuilder == null)
			{
				NewPathFinder.PfProfilerBeginSample("Distance Map Init");
				distanceBuilder = new RegionLinkDijkstra(map, targetCell, rootRegions, startCell, traverseParms, pathCosts);
				NewPathFinder.PfProfilerEndSample();
			}

			if (region.id != lastRegionId) //Cache the most recently retrieved region, since fetches will tend to be clustered.
			{
				NewPathFinder.PfProfilerBeginSample("Get Region Distance"); 
                bestLinkCost = distanceBuilder.GetRegionBestDistances(region, out bestLink, out secondBestLink, out secondBestLinkCost);
				lastRegionTilePathCost = distanceBuilder.RegionMinimumPathCost(region);
				NewPathFinder.PfProfilerEndSample();
				lastRegionId = region.id;
			}

			if (bestLink != null)
			{
				var costToLink = distanceBuilder.RegionLinkDistance(cell, bestLink, lastRegionTilePathCost);
				if (secondBestLink != null)
				{
					var costToSecondLink = distanceBuilder.RegionLinkDistance(cell, secondBestLink, lastRegionTilePathCost);
					return Math.Min(secondBestLinkCost + costToSecondLink, bestLinkCost + costToLink);
				}
				return bestLinkCost + costToLink;
			}

			return 1000000; //shouldn't happen except for sappers
		}

		private int OctileDistance(int dx, int dz) => (moveTicksCardinal * (dx + dz) + (moveTicksDiagonal - 2 * moveTicksCardinal) * Math.Min(dx, dz));
	}
}
