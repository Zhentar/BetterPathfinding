using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Linq.Expressions;
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

		public readonly Region[] regionGrid;

		private readonly IEnumerable<Region> rootRegions;

		private RegionLinkDijkstra distanceBuilder;

		private int lastRegionId = -1;
		private RegionLinkPathCostInfo bestLink;
		private RegionLinkPathCostInfo? secondBestLink;
        private TraverseParms traverseParms;

		private static Func<RegionGrid, Region[]> regionGridGetter;

		public RegionPathCostHeuristic(IntVec3 start, CellRect end, IEnumerable<Region> destRegions, TraverseParms parms, int cardinal, int diagonal)
		{
			startCell = start;
			targetCell = end.CenterCell;
			moveTicksCardinal = cardinal;
			moveTicksDiagonal = diagonal;

			var gridFunc = regionGridGetter ?? (regionGridGetter = GetFieldAccessor<RegionGrid, Region[]>("regionGrid"));
			regionGrid = regionGridGetter(Find.RegionGrid);
			rootRegions = new HashSet<Region>(destRegions);
            traverseParms = parms;
		}

		public static Func<TObject, TValue> GetFieldAccessor<TObject, TValue>(string fieldName)
		{
			ParameterExpression param = Expression.Parameter(typeof(TObject), "arg");
			MemberExpression member = Expression.Field(param, fieldName);
			LambdaExpression lambda = Expression.Lambda(typeof(Func<TObject, TValue>), member, param);
			Func<TObject, TValue> compiled = (Func<TObject, TValue>)lambda.Compile();
			return compiled;
		}

		public int GetPathCostToRegion(int cellIndex)
		{
			var region = regionGrid[cellIndex];
			var cell = CellIndices.IndexToCell(cellIndex);

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
