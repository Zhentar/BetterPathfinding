using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using UnityEngine;
using Verse;

namespace BetterPathfinding
{
	class RegionPathCostHeuristic
	{
	
		private IntVec3 startCell;
		private IntVec3 targetCell;
		private readonly int moveTicksCardinal;
		private readonly int moveTicksDiagonal;

		private readonly RegionGrid regionGrid;

		private readonly IEnumerable<Region> rootRegions;

		private RegionLinkDijkstra distanceBuilder;

		private int lastRegionId = -1;
		private RegionLinkPathCostInfo bestLink;
		private RegionLinkPathCostInfo? secondBestLink;
        private TraverseParms traverseParms;

		public RegionPathCostHeuristic(IntVec3 start, CellRect end, IEnumerable<Region> destRegions, TraverseParms parms, int cardinal, int diagonal)
		{
			startCell = start;
			targetCell = end.CenterCell;
			moveTicksCardinal = cardinal;
			moveTicksDiagonal = diagonal;
			regionGrid = Find.RegionGrid;
			rootRegions = new HashSet<Region>(destRegions);
            traverseParms = parms;
		}

		public int GetPathCostToRegion(int cellIndex)
		{
			var cell = CellIndices.IndexToCell(cellIndex);

			var region = regionGrid.GetValidRegionAt_NoRebuild(cell);

			if (rootRegions.Contains(region))
			{
				var dx = Mathf.Abs(cell.x - targetCell.x);
				var dz = Mathf.Abs(cell.z - targetCell.z);
				return OctileDistance(dx, dz);
			}

			if (distanceBuilder == null)
			{
				PathFinder.PfProfilerBeginSample("Distance Map Init");
				distanceBuilder = new RegionLinkDijkstra(targetCell, rootRegions, startCell, traverseParms, OctileDistance);
				PathFinder.PfProfilerEndSample();
			}

			if (region.id != lastRegionId) //Cache the most recently retrieved region, since fetches will tend to be clustered.
			{
				PathFinder.PfProfilerBeginSample("Get Region Distance");
				bestLink = distanceBuilder.GetNextRegionOverDistance(region, out secondBestLink);
				PathFinder.PfProfilerEndSample();
				lastRegionId = region.id;
			}

			if (bestLink.minLink != null)
			{
				var cost = GetTotalCost(bestLink, cell);
				if (!secondBestLink.HasValue)
				{
					return cost;
				}
				var secondCost = GetTotalCost(secondBestLink.Value, cell);
				
				return Mathf.Min(cost, secondCost);
			}

			return 1000000; //shouldn't happen except for sappers
		}

		private int GetTotalCost(RegionLinkPathCostInfo costInfo, IntVec3 cell)
		{
			int costToLink;
			if (costInfo.isDifferentRegionsLink)
			{
				costToLink = RegionLinkDijkstra.RegionLinkCenterDistance(cell, costInfo.minLink, OctileDistance, costInfo.minTilePathCost);
			}
			else
			{
				costToLink = RegionLinkDijkstra.RegionLinkDistance(cell, costInfo.minLink, OctileDistance, costInfo.minTilePathCost);
			}
			return costInfo.cost + costToLink;
		}
		
		private int OctileDistance(int dx, int dz) => (moveTicksCardinal * (dx + dz) + (moveTicksDiagonal - 2 * moveTicksCardinal) * Mathf.Min(dx, dz));
	}
}
