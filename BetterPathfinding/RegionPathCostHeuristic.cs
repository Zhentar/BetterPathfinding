using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Verse;

namespace BetterPathfinding
{

	// This class is pretty much just a mostly hollow shell wrapped around the actual heuristic calculating object.
	// At this point, pretty much the only purpose of it is to avoid instantiating the RegionLinkDijkstra for very short paths
	class RegionPathCostHeuristic
	{
	
		private readonly IntVec3 startCell;
		private IntVec3 targetCell;
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
		private Region[] regionGrid => regionGridGet(this.map.regionGrid);


		public RegionPathCostHeuristic(Map map, IntVec3 start, CellRect end, IEnumerable<Region> destRegions, TraverseParms parms, NewPathFinder.PawnPathCostSettings pathCosts)
		{
			this.map = map;
			startCell = start;
			targetCell = end.CenterCell;
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
				lastRegionTilePathCost = distanceBuilder.RegionMedianPathCost(region);
				NewPathFinder.PfProfilerEndSample();
				lastRegionId = region.id;
			}

			if (bestLink != null)
			{
				var costToLink = distanceBuilder.RegionLinkDistance(cell, bestLink, lastRegionTilePathCost);
				//Diagonal paths in open terrain often have two edges with similar costs; picking the best out of the two improves
				//the accuracy of the heuristic for the path, reducing reopens when the heuristic was wrong about which edge was really cheaper
				if (secondBestLink != null)
				{
					var costToSecondLink = distanceBuilder.RegionLinkDistance(cell, secondBestLink, lastRegionTilePathCost);
					return Math.Min(secondBestLinkCost + costToSecondLink, bestLinkCost + costToLink);
				}
				return bestLinkCost + costToLink;
			}

			return 1000000; //shouldn't happen except for sappers
		}

		private int OctileDistance(int dx, int dz) => (pathCosts.moveTicksCardinal * (dx + dz) + (pathCosts.moveTicksDiagonal - 2 * pathCosts.moveTicksCardinal) * Math.Min(dx, dz));
	}
}
