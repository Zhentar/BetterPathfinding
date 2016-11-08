using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using RimWorld;
using UnityEngine;
using Verse;

namespace BetterPathfinding
{

	public struct RegionLinkPathCostInfo
	{
		public int cost;
		public RegionLink minLink;
		public int minTilePathCost;
		public bool isDifferentRegionsLink;
	}

	public class RegionLinkDijkstra
	{

		private struct RegionLinkQueueEntry
		{
			public Region FromRegion;
			public RegionLink Link;
			public int Cost;
            public int EstimatedPathCost;

			public RegionLinkQueueEntry(Region from, RegionLink l, int c, int tc)
			{
				FromRegion = from;
				Link = l;
				Cost = c;
                EstimatedPathCost = tc;
			}
		}


		private class DistanceComparer : IComparer<RegionLinkQueueEntry>
		{
			public int Compare(RegionLinkQueueEntry a, RegionLinkQueueEntry b)
			{
				return a.EstimatedPathCost.CompareTo(b.EstimatedPathCost);
			}
		}

		private readonly Dictionary<int, RegionLink> regionMinLink = new Dictionary<int, RegionLink>();

		private readonly Dictionary<RegionLink, int> distances = new Dictionary<RegionLink, int>();

		private readonly FastPriorityQueue<RegionLinkQueueEntry> queue = new FastPriorityQueue<RegionLinkQueueEntry>(new DistanceComparer());

		private Func<int, int, int> costCalculator;

        private TraverseParms traverseParms;

        private IntVec3 targetCell;

		private ByteGrid avoidGrid;

		public static int nodes_popped;

		private Dictionary<Region, int> minPathCosts = new Dictionary<Region, int>();

		public RegionLinkDijkstra(IntVec3 rootCell, IEnumerable<Region> startingRegions, IntVec3 target, TraverseParms parms, Func<int, int, int> cost)
		{
			this.costCalculator = cost;
            this.traverseParms = parms;
            this.targetCell = target;
			avoidGrid = parms.pawn?.GetAvoidGrid();
			nodes_popped = 0;
			
			foreach (var region in startingRegions)
			{
				var minPathCost = RegionMinimumPathCost(region);
				foreach (RegionLink current in region.links) 
				{
					var dist = RegionLinkDistance(rootCell, current, cost, minPathCost);
					if (distances.ContainsKey(current))
					{
						 dist = Math.Min(distances[current], dist);
					}
					distances[current] = dist;
					queue.Push(new RegionLinkQueueEntry(region, current, dist, dist));
				}
			}
		}

		public RegionLinkPathCostInfo GetNextRegionOverDistance(Region region, out RegionLinkPathCostInfo? secondBest)
		{
			secondBest = null;
			RegionLinkPathCostInfo result = new RegionLinkPathCostInfo();
			RegionLink minLink;
			result.cost = GetRegionDistance(region, out minLink);
			RegionLinkPathCostInfo(region, minLink, ref result);

			RegionLink secondBestLink = null;
			int secondBestCost = Int32.MaxValue;

			foreach (var link in region.links)
			{
				if(link == minLink) { continue; }

				if (distances.ContainsKey(link))
				{
					var cost = distances[link];
					if (cost < secondBestCost)
					{
						secondBestCost = cost;
						secondBestLink = link;
					}
				}
			}

			if (secondBestLink != null)
			{
				var secondResult = new RegionLinkPathCostInfo();
				secondResult.cost = secondBestCost;
				RegionLinkPathCostInfo(region, secondBestLink, ref secondResult);
				secondBest = secondResult;
			}

			return result;
		}

		private void RegionLinkPathCostInfo(Region region, RegionLink minLink, ref RegionLinkPathCostInfo result)
		{
			result.minLink = minLink;
			result.minTilePathCost = RegionMinimumPathCost(region);
			//In open terrain, returning the region link past the first helps smooth out discontinuities (reducing reopened nodes)
			//In tighter areas, returning the next region link over hides the cost of obstructions, and
			//shorter spans have smaller discontinuities so they cause fewer re-opens anyway
			if (minLink.span.length < 9)
			{
				return;
			}
			var secondRegion = GetLinkOtherRegion(region, minLink);
			if (!regionMinLink.ContainsKey(secondRegion.id)) //it's the destination region
			{
				return;
			}
			result.minTilePathCost = Mathf.Min(result.minTilePathCost, RegionMinimumPathCost(secondRegion));
			result.minLink = regionMinLink[secondRegion.id];
			result.isDifferentRegionsLink = true;
			result.cost = distances[result.minLink];
		}

		public int GetRegionDistance(Region region, out RegionLink minLink)
		{
			if (regionMinLink.TryGetValue(region.id, out minLink))
			{
				return distances[minLink];
			}

			while (queue.Count != 0) {
				var vertex = queue.Pop();
				nodes_popped++;
				int knownBest = distances[vertex.Link];
				if (vertex.Cost == knownBest) //Will this ever not be true? - Yes. Not sure why. 
                {
                    var destRegion = GetLinkOtherRegion(vertex.FromRegion, vertex.Link);

					//I've never encountered this during testing, but users reported issues resolved by this check.
	                if (destRegion?.valid != true) { continue; }

                    //TODO: lying about destination to avoid danger check... should it work this way?
                    if (destRegion.portal != null && !destRegion.Allows(traverseParms, true))
                    {
                        continue;
                    }

					var minPathCost = RegionMinimumPathCost(destRegion);
					foreach (var current2 in destRegion.links)
					{
						if (current2 == vertex.Link) { continue; }
						var addedCost = destRegion.portal != null ? GetPortalCost(destRegion.portal) : RegionLinkDistance(vertex.Link, current2, minPathCost);
						int newCost = knownBest + addedCost;
                        int pathCost = RegionLinkDistance(targetCell, current2, costCalculator, 0) + newCost;
						int oldCost;
						if (distances.TryGetValue(current2, out oldCost))
						{
							if (newCost < oldCost)
							{
								distances[current2] = newCost;
								queue.Push(new RegionLinkQueueEntry(destRegion, current2, newCost, pathCost));
							}
						}
						else
						{
							distances.Add(current2, newCost);
							queue.Push(new RegionLinkQueueEntry(destRegion, current2, newCost, pathCost));
						}
					}
					//if this is the first vertex popped for the region, we've found the shortest path to that region
					if (!regionMinLink.ContainsKey(destRegion.id)) {
						regionMinLink[destRegion.id] = vertex.Link;
						if (destRegion.id == region.id) {
							minLink = vertex.Link;
							return vertex.Cost;
						}
					}
				}
			}

			return 100000;
		}

		//If we just use the pawn move speed as the path cost, we're effectively telling A* "there might be a road around here somewhere, keep looking!"
		//This makes it expand a whole lot more nodes than necessary in open, rough terrain, searching high and low for that alleged road.
		//Finding the minimum path cost of any tile in the region is a cheap way to guess if that road could possibly exist.
		//This could be cached across pathfinding calls, but I'd need extra detours to invalidate it and it apparently performs adequately without it.
		public int RegionMinimumPathCost(Region region)
		{
			int minCost;
			if (minPathCosts.TryGetValue(region, out minCost))
			{
				return minCost;
			}
			minCost = 10000;

			for (int z = region.extentsClose.minZ; z <= region.extentsClose.maxZ; z++) {
				for (int x = region.extentsClose.minX; x <= region.extentsClose.maxX; x++)
				{
					int index = CellIndices.CellToIndex(x, z);
					minCost = Mathf.Min(minCost, Find.PathGrid.pathGrid[index] + (avoidGrid?[index]*8 ?? 0));
					if (minCost == 0)
					{
						minPathCosts[region] = 0;
						return 0;
					}
				}
			}
			minPathCosts[region] = minCost;

			return minCost;
		}

		private int GetPortalCost(Building_Door portal)
		{
			return portal.TicksToOpenNow + costCalculator(1,0);
		}

		private int RegionLinkDistance(RegionLink a, RegionLink b, int minPathCost)
		{
			int dx = Mathf.Abs(SpanCenterX(a.span) - SpanCenterX(b.span));
			int dz = Mathf.Abs(SpanCenterZ(a.span) - SpanCenterZ(b.span));

			return costCalculator(dx, dz) + minPathCost * Mathf.Max(dx, dz);
		}

		private static int SpanCenterX(EdgeSpan e) => e.root.x + (e.dir == SpanDirection.East ? e.length / 2 : 0);

		private static int SpanCenterZ(EdgeSpan e) => e.root.z + (e.dir == SpanDirection.North ? e.length / 2 : 0);


		private static int SpanEndX(EdgeSpan e) => e.root.x + (e.dir == SpanDirection.East ? e.length : 0);

		private static int SpanEndZ(EdgeSpan e) => e.root.z + (e.dir == SpanDirection.North ? e.length : 0);

		public static int RegionLinkCenterDistance(IntVec3 cell, RegionLink link, Func<int, int, int> cost, int minPathCost)
		{
			int dx = Mathf.Abs(cell.x - SpanCenterX(link.span));
			int dz = Mathf.Abs(cell.z - SpanCenterZ(link.span));

			//int dx = Math.Max(Mathf.Abs(cell.x - SpanEndX(link.span)), Mathf.Abs(cell.x - link.span.root.x));
			//int dz = Math.Max(Mathf.Abs(cell.z - SpanEndZ(link.span)), Mathf.Abs(cell.z - link.span.root.z));

			return cost(dx, dz) + minPathCost * Mathf.Max(dx, dz);
		}

		public static int RegionLinkDistance(IntVec3 cell, RegionLink link, Func<int, int, int> cost, int minPathCost)
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
            return cost(dx, dz) + minPathCost * Mathf.Max(dx, dz);
        }

        private static int GetValue(int cellz, int spanz, int spanLen) => cellz < spanz ? spanz - cellz : Mathf.Max(cellz - (spanz + spanLen), 0);

        private static Region GetLinkOtherRegion(Region fromRegion, RegionLink link) =>  Equals(fromRegion, link.RegionA) ? link.RegionB : link.RegionA;
    }
}
