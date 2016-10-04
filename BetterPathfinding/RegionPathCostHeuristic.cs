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

		private readonly Region rootRegion;

		private RegionLinkDijkstra distanceBuilder;

		private int lastRegionId = -1;
		private RegionLink minLink;
		private int minCost = -1;

		public static Stopwatch DijkstraStopWatch;

		public RegionPathCostHeuristic(IntVec3 start, IntVec3 end, int cardinal, int diagonal)
		{
			startCell = start;
			targetCell = end;
			moveTicksCardinal = cardinal;
			moveTicksDiagonal = diagonal;
			regionGrid = Find.RegionGrid;
			rootRegion = regionGrid.GetValidRegionAt(end);
		}

		public int GetPathCostToRegion(int cellIndex)
		{
			var cell = CellIndices.IndexToCell(cellIndex);

			var region = regionGrid.GetValidRegionAt_NoRebuild(cell);

			if (region.Equals(rootRegion))
			{
				var dx = Mathf.Abs(cell.x - targetCell.x);
				var dz = Mathf.Abs(cell.z - targetCell.z);
				return OctileDistance(dx, dz);
			}

			if (distanceBuilder == null) {
#if DEBUG
				DijkstraStopWatch = Stopwatch.StartNew();
#endif
				distanceBuilder = new RegionLinkDijkstra(targetCell, OctileDistance, RegionLinkDistance);
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
#if DEBUG
				DijkstraStopWatch.Stop();
#endif
				lastRegionId = region.id;
			}

			if (minLink != null)
			{
				var linkCost = RegionLinkDistance(cell, minLink);
				return minCost + linkCost;
			}

			return 1000000; //shouldn't happen except for sappers
		}

		//TODO: calculate to the closest point of the edge
		private int RegionLinkDistance(IntVec3 cell, RegionLink link)
		{
			int dx, dz;
			if (link.span.dir == SpanDirection.North)
			{
				dx = Mathf.Abs(cell.x - link.span.root.x);
				dz = GetValue(cell.z, link.span.root.z, link.span.length);
			}
			else
			{
				dz = Mathf.Abs(cell.z - link.span.root.z);
				dx = GetValue(cell.x, link.span.root.x, link.span.length);
			}
			return OctileDistance(dx, dz);
		}

		private static int GetValue(int cellz, int spanz, int spanLen) => cellz < spanz ? spanz - cellz : Mathf.Max(cellz - (spanz + spanLen), 0);

		private int OctileDistance(int dx, int dz) => (moveTicksCardinal * (dx + dz) + (moveTicksDiagonal - 2 * moveTicksCardinal) * Mathf.Min(dx, dz));
	}
}
