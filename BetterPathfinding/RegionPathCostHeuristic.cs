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
		private RegionLink minLink;
		private int minCost = -1;
        private int minPathCost = 0;
        private TraverseParms traverseParms;

        public static Stopwatch DijkstraStopWatch;

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

			if (distanceBuilder == null) {
#if DEBUG
				DijkstraStopWatch = Stopwatch.StartNew();
#endif
				distanceBuilder = new RegionLinkDijkstra(targetCell, rootRegions, startCell, traverseParms, OctileDistance);
#if DEBUG
				DijkstraStopWatch.Stop();
#endif
			}

			if (region.id != lastRegionId) //Cache the most recently retrieved region, since fetches will tend to be clustered.
			{
#if DEBUG
				DijkstraStopWatch.Start();
#endif
				minCost = distanceBuilder.GetRegionDistance(region, out minLink);
				minPathCost = distanceBuilder.RegionMinimumPathCost(region);
#if DEBUG
				DijkstraStopWatch.Stop();
#endif
				lastRegionId = region.id;
			}

			if (minLink != null)
			{
                var linkCost = RegionLinkDijkstra.RegionLinkDistance(cell, minLink, OctileDistance, minPathCost);
				return minCost + linkCost;
			}

			return 1000000; //shouldn't happen except for sappers
		}
		
		private int OctileDistance(int dx, int dz) => (moveTicksCardinal * (dx + dz) + (moveTicksDiagonal - 2 * moveTicksCardinal) * Mathf.Min(dx, dz));
	}
}
