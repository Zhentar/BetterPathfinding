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
			public readonly Region FromRegion;
			public readonly RegionLink Link;
			public readonly int Cost;
            public readonly int EstimatedPathCost;

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

		private readonly Func<int, int, int> costCalculator;

        private TraverseParms traverseParms;

        private readonly IntVec3 targetCell;

		private readonly ByteGrid avoidGrid;

		private readonly Area area;

		private readonly Map map;

		public static int nodes_popped;

		private readonly Dictionary<Region, int> minPathCosts = new Dictionary<Region, int>();

		//private NewPathFinder debugPathfinder;
		public IntVec3 rootCell;

		public RegionLinkDijkstra(Map map, IntVec3 rootCell, IEnumerable<Region> startingRegions, IntVec3 target, TraverseParms parms, Func<int, int, int> cost)
		{
			this.map = map;
			this.costCalculator = cost;
            this.traverseParms = parms;
            this.targetCell = target;
			this.rootCell = rootCell;
			avoidGrid = parms.pawn?.GetAvoidGrid();
			if (parms.pawn?.Drafted == false)
			{
				area = parms.pawn?.playerSettings?.AreaRestrictionInPawnCurrentMap;
			}
			nodes_popped = 0;

			//if (DebugViewSettings.drawPaths && !NewPathFinder.disableDebugFlash)
			//{
			//	debugPathfinder = new NewPathFinder(map);
			//}

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

					if (destRegion.portal != null)
					{
						//Not using Region.Allows because it is not entirely consistent with the pathfinder logic
						//Resulting in errors when a door is within range of 22 turrets
						switch (traverseParms.mode)
						{
							case TraverseMode.ByPawn:
								if (!traverseParms.canBash && destRegion.portal.IsForbiddenToPass(traverseParms.pawn))
								{
									continue;
								}
								if (!destRegion.portal.FreePassage && !destRegion.portal.PawnCanOpen(traverseParms.pawn) &&
									!traverseParms.canBash)
								{
									continue;
								}
								break;
							case TraverseMode.NoPassClosedDoors:
								if (!destRegion.portal.FreePassage)
								{
									continue;
								}
								break;
						}
					}



					var minPathCost = RegionMinimumPathCost(destRegion);
					foreach (var current2 in destRegion.links)
					{
						if (current2 == vertex.Link) { continue; }
						var addedCost = destRegion.portal != null ? GetPortalCost(destRegion.portal) : RegionLinkDistance(vertex.Link, current2, minPathCost);
						addedCost = Math.Max(addedCost, 1); //Handle mods with negative path costs
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
						//if (DebugViewSettings.drawPaths && !NewPathFinder.disableDebugFlash)
						//{
						//	NewPathFinder.disableDebugFlash = true;
						//	var tempPath = debugPathfinder?.FindPathInner(this.rootCell, new LocalTargetInfo(RegionLinkCenter(vertex.Link)), this.traverseParms, Verse.AI.PathEndMode.OnCell);
						//	NewPathFinder.disableDebugFlash = false;
						//	var actualCost = tempPath.TotalCost;
						//	tempPath.Dispose();
						//	if (regionMinLink.TryGetValue(vertex.FromRegion.id, out minLink))
						//		NewPathFinder.DebugLine(this.map, RegionLinkCenter(vertex.Link), RegionLinkCenter(minLink));
						//	NewPathFinder.DebugFlash(this.map, RegionLinkCenter(vertex.Link), knownBest/1500f, knownBest + "\n(" + actualCost + ")");
						//	if (actualCost < knownBest) { Log.Warning(vertex.Link + " has actual cost " + actualCost + "with heuristic " + knownBest); }
						//}
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
					var index = this.map.cellIndices.CellToIndex(x, z);
					var cellCost = this.map.pathGrid.pathGrid[index] + (avoidGrid?[index]*8 ?? 0);
					if (area != null && !area[index])
					{
						cellCost += 600;
					}
					minCost = Mathf.Min(minCost, cellCost);
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

		private static IntVec3 RegionLinkCenter(RegionLink link) => new IntVec3(SpanCenterX(link.span), 0, SpanCenterZ(link.span));

		private static int SpanEndX(EdgeSpan e) => e.root.x + (e.dir == SpanDirection.East ? e.length : 0);

		private static int SpanEndZ(EdgeSpan e) => e.root.z + (e.dir == SpanDirection.North ? e.length : 0);

		public static int RegionLinkCenterDistance(IntVec3 cell, RegionLink link, Func<int, int, int> cost, int minPathCost)
		{
			int dx = Mathf.Abs(cell.x - SpanCenterX(link.span));
			int dz = Mathf.Abs(cell.z - SpanCenterZ(link.span));

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
