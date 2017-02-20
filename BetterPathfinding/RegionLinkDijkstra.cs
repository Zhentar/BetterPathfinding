using System;
using System.Collections.Generic;
using System.Linq;
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

		//The idea of this was to get a more accurate target cell than the region link center when the destination is close to the edge of a region
		//Using it as a target in the cell -> regionlink distance calculation was a disaster. But it helps the initial regionlink -> regionlink
		//search enough to be worth keeping around.
	    private readonly Dictionary<RegionLink, IntVec3> linkTargetCells = new Dictionary<RegionLink, IntVec3>();

		private readonly FastPriorityQueue<RegionLinkQueueEntry> queue = new FastPriorityQueue<RegionLinkQueueEntry>(new DistanceComparer());
		
        private readonly TraverseParms traverseParms;

        private readonly IntVec3 targetCell;

		private readonly ByteGrid avoidGrid;

		private readonly Area area;

		private readonly Map map;

		private readonly Region[] regionGrid;

	    private readonly NewPathFinder.PawnPathCostSettings pathCostSettings;

        public static int nodes_popped;

		private readonly Dictionary<Region, int> minPathCosts = new Dictionary<Region, int>();

		//private NewPathFinder debugPathfinder;
		public IntVec3 rootCell;

		public RegionLinkDijkstra(Map map, IntVec3 rootCell, IEnumerable<Region> startingRegions, IntVec3 target, TraverseParms parms, NewPathFinder.PawnPathCostSettings pathCosts)
		{
			this.map = map;
			this.regionGrid = RegionPathCostHeuristic.regionGridGet(map.regionGrid);
            this.traverseParms = parms;
            this.targetCell = target;
			this.rootCell = rootCell;
		    this.pathCostSettings = pathCosts;
			avoidGrid = pathCosts.avoidGrid;
			area = pathCosts.area;
			nodes_popped = 0;
			
			//if (DebugViewSettings.drawPaths && !NewPathFinder.disableDebugFlash)
			//{
			//	debugPathfinder = new NewPathFinder(map);
			//}

			//Init the starting region links from the start cell
			foreach (var region in startingRegions)
			{
				var minPathCost = RegionMedianPathCost(region);
				foreach (RegionLink current in region.links) 
				{
					var dist = RegionLinkDistance(rootCell, current, minPathCost);
					if (distances.ContainsKey(current))
					{
						if (dist < distances[current])
						{
							linkTargetCells[current] = GetLinkTargetCell(rootCell, current);
						}
						dist = Math.Min(distances[current], dist);
					}
					else
					{ linkTargetCells[current] = GetLinkTargetCell(rootCell, current); }
					distances[current] = dist;
				}
				foreach (var pair in PreciseRegionLinkDistances(region, rootCell))
				{
					var current = pair.First;
					var dist = Math.Max(pair.Second, distances[current]);
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
				if (vertex.Cost == knownBest)
                {
                    var destRegion = GetLinkOtherRegion(vertex.FromRegion, vertex.Link);

					//I've never encountered this during testing, but users reported issues resolved by this check.
	                if (destRegion?.valid != true) { continue; }

                    int portalCost = 0;
					if (destRegion.portal != null)
                    {
                        //Not using Region.Allows because it is not entirely consistent with the pathfinder logic,
                        //resulting in errors when a door is within range of 22 turrets (avoidGrid=255)
                        portalCost = NewPathFinder.GetPathCostForBuilding(destRegion.portal, traverseParms);
                        if(portalCost < 0 ) { continue; }
                        portalCost = portalCost + OctileDistance(1, 0);
                    }

					var minPathCost = RegionMedianPathCost(destRegion);
					foreach (var current2 in destRegion.links)
					{
						if (current2 == vertex.Link) { continue; }
						var addedCost = destRegion.portal != null ? portalCost : RegionLinkDistance(vertex.Link, current2, minPathCost);
						addedCost = Math.Max(addedCost, 1); //Handle mods with negative path costs
						int newCost = knownBest + addedCost;
                        int pathCost = MinimumRegionLinkDistance(targetCell, current2) + newCost;
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
						//	//NewPathFinder.disableDebugFlash = true;
						//	//var tempPath = debugPathfinder?.FindPathInner(this.rootCell, new LocalTargetInfo(RegionLinkCenter(vertex.Link)), this.traverseParms, Verse.AI.PathEndMode.OnCell);
						//	//NewPathFinder.disableDebugFlash = false;
						//	//var actualCost = tempPath.TotalCost;
						//	//tempPath.Dispose();
						//	if (regionMinLink.TryGetValue(vertex.FromRegion.id, out minLink))
						//		NewPathFinder.DebugLine(this.map, RegionLinkCenter(vertex.Link), RegionLinkCenter(minLink));
						//	NewPathFinder.DebugFlash(this.map, RegionLinkCenter(vertex.Link), knownBest / 1500f, $"{knownBest}\n{nodes_popped}" /*+ "\n(" + actualCost + ")"*/);
						//	//if (actualCost < knownBest) { Log.Warning(vertex.Link + " has actual cost " + actualCost + "with heuristic " + knownBest); }
						//}
						regionMinLink[destRegion.id] = vertex.Link;
						if (destRegion.id == region.id)
						{
							minLink = vertex.Link;
							return vertex.Cost;
						}
					}
				}
			}

			return 100000;
		}


		public int GetRegionBestDistances(Region region, out RegionLink bestLink, out RegionLink secondBestLink, out int secondBestCost)
		{
			int bestCost = GetRegionDistance(region, out bestLink);

			secondBestLink = null;
			secondBestCost = Int32.MaxValue;

			foreach (var link in region.links)
			{
				if (link == bestLink) { continue; }

				int cost;
				if (distances.TryGetValue(link, out cost) && cost < secondBestCost)
				{
					secondBestCost = cost;
					secondBestLink = link;
				}
			}

			return bestCost;
		}


		//If we just use the pawn move speed as the path cost, we're effectively telling A* "there might be a road around here somewhere, keep looking!"
		//This makes it expand a whole lot more nodes than necessary in open, rough terrain, searching high and low for that alleged road.
		//Finding the minimum path cost of any tile in the region is a cheap way to guess if that road could possibly exist.
		//This could be cached across pathfinding calls, but I'd need extra detours to invalidate it and it apparently performs adequately without it.
		//
		// Minimum becomes Median: The minimum path cost preserved admissibility, but it tends to underestimate; small patches of low cost terrain among
		// very high cost terrain cause significant underestimates and many extra nodes explored. The median is much, much more accurate in most cases
		// Some of my test cases opened just 1/5-1/10 as many nodes compared to the minimum. Just one problem: calculating the true median is expensive.
		// Random sampling gets reasonably good accuracy at a much lower cost. 
		// 11 samples scientifically determined by untested guess.
		public int RegionMedianPathCost(Region region)
		{
			int minCost;
			if (minPathCosts.TryGetValue(region, out minCost))
			{
				return minCost;
			}

			//Setting a seed for deterministic behavior so test cases are consistent and bad paths can be reproduced
			Rand.PushSeed();
			Rand.Seed = map.cellIndices.CellToIndex(region.extentsClose.CenterCell) * (region.links.Count + 1);
			for (int i = 0; i < sampleCount; i++) { pathCostSamples[i] = GetCellCostFast(map.cellIndices.CellToIndex(region.RandomCell)); }
			Rand.PopSeed();
			Array.Sort(pathCostSamples);

			// -3 because when a region is half full of something expensive (chunks, hydroponics bays), there's probably a cheaper path around them.
			// Intentionally underestimating does hurt performance a bit, but it can also improve path quality significantly.
			return minPathCosts[region] = pathCostSamples[(sampleCount-3)/2];
		}

		private static readonly int[] pathCostSamples = new int[sampleCount];

		private const int sampleCount = 11;


		private int GetCellCostFast(int index)
		{
			var cellCost = this.map.pathGrid.pathGrid[index] + (avoidGrid?[index] * 8 ?? 0);
			if (area?[index] == false) { cellCost = (Math.Max(cellCost,10) + pathCostSettings.moveTicksCardinal * 2) * 10; }
			return cellCost;
		}

		private int RegionLinkDistance(RegionLink a, RegionLink b, int minPathCost)
		{
		    var aCell = linkTargetCells.ContainsKey(a) ? linkTargetCells[a] : RegionLinkCenter(a);
            var bCell = linkTargetCells.ContainsKey(b) ? linkTargetCells[b] : RegionLinkCenter(b);
            var diff = aCell - bCell;
            var dx = Math.Abs(diff.x);
            var dz = Math.Abs(diff.z);

            return OctileDistance(dx, dz) + minPathCost * Math.Max(dx, dz) + (int)(minPathCost * Math.Min(dx, dz) * (NewPathFinder.diagonalPerceivedCostWeight - 1.0f));
		}

		private static int SpanCenterX(EdgeSpan e) => e.root.x + (e.dir == SpanDirection.East ? e.length / 2 : 0);

		private static int SpanCenterZ(EdgeSpan e) => e.root.z + (e.dir == SpanDirection.North ? e.length / 2 : 0);

		private static IntVec3 RegionLinkCenter(RegionLink link) => new IntVec3(SpanCenterX(link.span), 0, SpanCenterZ(link.span));

		private static int SpanEndX(EdgeSpan e) => e.root.x + (e.dir == SpanDirection.East ? e.length : 0);

		private static int SpanEndZ(EdgeSpan e) => e.root.z + (e.dir == SpanDirection.North ? e.length : 0);

		public static int RegionLinkCenterDistance(IntVec3 cell, RegionLink link, Func<int, int, int> cost, int minPathCost)
		{
			int dx = Math.Abs(cell.x - SpanCenterX(link.span));
			int dz = Math.Abs(cell.z - SpanCenterZ(link.span));

			return cost(dx, dz) + minPathCost * Math.Max(dx, dz);
		}

		private int MinimumRegionLinkDistance(IntVec3 cell, RegionLink link)
		{
			var diff = cell - LinkClosestCell(cell, link);
			return OctileDistance(Math.Abs(diff.x), Math.Abs(diff.z));

		}

		public int RegionLinkDistance(IntVec3 cell, RegionLink link, int minPathCost)
		{
		    var targetCell = GetLinkTargetCell(cell, link);

		    var diff = cell - targetCell;
		    var dx = Math.Abs(diff.x);
            var dz = Math.Abs(diff.z);

            return OctileDistance(dx, dz) + minPathCost * Math.Max(dx, dz) + (int)(minPathCost * Math.Min(dx, dz) * (NewPathFinder.diagonalPerceivedCostWeight - 1.0f));
        }

		private int OctileDistance(int dx, int dz) => (pathCostSettings.moveTicksCardinal * (dx + dz) + (pathCostSettings.moveTicksDiagonal - 2 * pathCostSettings.moveTicksCardinal) * Math.Min(dx, dz));

		//The idea here is that for cells far away from a regionlink, we estimate the cost to the center of the regionlink (same as the regionlink to regionlink estimates)
		//But then closer to the regionlink, the path straight towards the link gets used (so the region corners don't get unreasonably high cost estimates).
		//It helps some of my test cases quite a bit (hurts a couple too, though), so it seems to be worth the effort.
		private IntVec3 GetLinkTargetCell(IntVec3 cell, RegionLink link)
	    {
		    var targetCell = LinkClosestCell(cell, link);

		    var diff = cell - targetCell;
            var dx = Math.Abs(diff.x);
            var dz = Math.Abs(diff.z);

            var dist = link.span.dir == SpanDirection.North ? dx : dz;
	        var factor = Math.Min(6, dist) / 6.0;
	        IntVec3 centerCell = RegionLinkCenter(link);

	        targetCell = IntVec3Lerp(targetCell, centerCell, factor);

            return targetCell;
	    }

		private static IntVec3 LinkClosestCell(IntVec3 cell, RegionLink link)
		{
			int width = 0;
			int height = 0;
			if (link.span.dir == SpanDirection.North) { height = link.span.length - 1; }
			else
			{ width = link.span.length - 1; }

			IntVec3 targetCell = new IntVec3(Mathf.Clamp(cell.x, link.span.root.x, link.span.root.x + width), 0, Mathf.Clamp(cell.z, link.span.root.z, link.span.root.z + height));
			return targetCell;
		}

		private static IntVec3 IntVec3Lerp(IntVec3 cellA, IntVec3 cellB, double factor)
	    {
	        return new IntVec3((int) Math.Round(cellA.x + (cellB.x - cellA.x) * factor), 0, (int) Math.Round(cellA.z + (cellB.z - cellA.z) * factor));
	    }

	    private static Region GetLinkOtherRegion(Region fromRegion, RegionLink link) =>  Equals(fromRegion, link.RegionA) ? link.RegionB : link.RegionA;


		private IEnumerable<Pair<RegionLink, int>> PreciseRegionLinkDistances(Region region, IntVec3 start)
		{
			var startIndex = map.cellIndices.CellToIndex(start);
			IEnumerable<int> startIndices = regionGrid[startIndex]?.id != region.id ? startIndex.NeighborIndices(map) : new[] {startIndex};
			
			Dictionary<int, float> outDistances = new Dictionary<int, float>();
			Dijkstra<int>.Run(startIndices,
									x => regionGrid[x]?.id != region.id ? Enumerable.Empty<int>() : x.PathableNeighborIndices(map),
									(a, b) => GetCellCostFast(b) + (a.IsIndexDiagonal(b, map) ? pathCostSettings.moveTicksDiagonal : pathCostSettings.moveTicksCardinal),
									ref outDistances);
			foreach (var link in region.links)
			{
				var minCost = outDistances[map.cellIndices.CellToIndex(linkTargetCells[link])];
				yield return new Pair<RegionLink, int>(link, (int)minCost);
			}
		}


    }
}
