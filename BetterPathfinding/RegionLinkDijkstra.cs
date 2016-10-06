using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using RimWorld;
using UnityEngine;
using Verse;

namespace BetterPathfinding
{
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

		public static int nodes_popped;

		public RegionLinkDijkstra(IntVec3 rootCell, IntVec3 target, TraverseParms parms, Func<int, int, int> cost)
		{
			this.costCalculator = cost;
            this.traverseParms = parms;
            this.targetCell = target;

			nodes_popped = 0;

			var rootRegion = Find.RegionGrid.GetValidRegionAt(rootCell);

			var startingRegions = new[] { rootRegion };
			foreach (var region in startingRegions) {
				foreach (RegionLink current in region.links) {
					var dist = RegionLinkDistance(rootCell, current, cost);
					distances.Add(current, dist);
					queue.Push(new RegionLinkQueueEntry(region, current, dist, dist));
				}
			}
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
                        int pathCost = RegionLinkDistance(targetCell, current2, costCalculator) + newCost;
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
		public static int RegionMinimumPathCost(Region region)
		{
			int minCost = 10000;

			for (int z = region.extentsClose.minZ; z <= region.extentsClose.maxZ; z++) {
				for (int x = region.extentsClose.minX; x <= region.extentsClose.maxX; x++)
				{
					int index = CellIndices.CellToIndex(x, z);
					minCost = Mathf.Min(minCost, Find.PathGrid.pathGrid[index]);
					if (minCost == 0) 
					{
						return 0;
					}
				}
			}

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

        public static int RegionLinkDistance(IntVec3 cell, RegionLink link, Func<int, int, int> cost)
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
            return cost(dx, dz);
        }

        private static int GetValue(int cellz, int spanz, int spanLen) => cellz < spanz ? spanz - cellz : Mathf.Max(cellz - (spanz + spanLen), 0);

        private static Region GetLinkOtherRegion(Region fromRegion, RegionLink link) =>  Equals(fromRegion, link.RegionA) ? link.RegionB : link.RegionA;
    }
}
