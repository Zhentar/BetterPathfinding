using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using RimWorld;
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

			public RegionLinkQueueEntry(Region from, RegionLink l, int c)
			{
				FromRegion = from;
				Link = l;
				Cost = c;
			}
		}


		private class DistanceComparer : IComparer<RegionLinkQueueEntry>
		{
			public int Compare(RegionLinkQueueEntry a, RegionLinkQueueEntry b)
			{
				return a.Cost.CompareTo(b.Cost);
			}
		}

		private readonly Dictionary<int, RegionLink> regionMinLink = new Dictionary<int, RegionLink>();

		private readonly Dictionary<RegionLink, int> distances = new Dictionary<RegionLink, int>();

		private readonly FastPriorityQueue<RegionLinkQueueEntry> queue = new FastPriorityQueue<RegionLinkQueueEntry>(new DistanceComparer());

		private Func<RegionLink, RegionLink, int> distanceGetter;
		private Func<IntVec3, RegionLink, int> cellDistanceGetter;

		public static int nodes_popped;

		public RegionLinkDijkstra(IntVec3 rootCell, Func<RegionLink, RegionLink, int> distance, Func<IntVec3, RegionLink, int> cellDistance)
		{
			this.distanceGetter = distance;
			this.cellDistanceGetter = cellDistance;

			nodes_popped = 0;

			var rootRegion = Find.RegionGrid.GetValidRegionAt(rootCell);

			var startingRegions = new[] { rootRegion };
			foreach (var region in startingRegions) {
				foreach (RegionLink current in region.links) {
					var dist = cellDistanceGetter(rootCell, current);
					distances.Add(current, dist);
					queue.Push(new RegionLinkQueueEntry(region, current, dist));
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
				var destRegion = Equals(vertex.FromRegion, vertex.Link.RegionA) ? vertex.Link.RegionB : vertex.Link.RegionA;
				if (vertex.Cost == knownBest) //Will this ever not be true? - Yes. Not sure why. 
				{
					//TODO: pass in the traverse parms so we can properly consider door restrictions
					foreach (var current2 in destRegion.links)
					{
						if (current2 == vertex.Link) { continue; }
						int addedCost;
						if (destRegion.portal != null)
						{
							addedCost = GetPortalCost(destRegion.portal);
						}
						else
						{
							addedCost = distanceGetter(vertex.Link, current2);
						}
						int newCost = knownBest + addedCost;
						int oldCost;
						if (distances.TryGetValue(current2, out oldCost))
						{
							if (newCost < oldCost)
							{
								distances[current2] = newCost;
								queue.Push(new RegionLinkQueueEntry(destRegion, current2, newCost));
							}
						}
						else
						{
							distances.Add(current2, newCost);
							queue.Push(new RegionLinkQueueEntry(destRegion, current2, newCost));
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

		private int GetPortalCost(Building_Door portal)
		{
			return portal.TicksToOpenNow + 13 /*TODO: moveCostCardinal*/;
		}
	}
}
