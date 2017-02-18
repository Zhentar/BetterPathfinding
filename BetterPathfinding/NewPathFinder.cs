using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using RimWorld;
using Verse;
using Verse.AI;

namespace BetterPathfinding
{
#region Injection Code
	static class PathFinderDetour
	{

		private static readonly Func<PathFinder, Map> mapGet = Utils.GetFieldAccessor<PathFinder, Map>("map");

		public static PawnPath FindPath(this PathFinder @this, IntVec3 start, LocalTargetInfo dest, TraverseParms traverseParms, PathEndMode peMode = PathEndMode.OnCell)
		{
			var map = mapGet(@this);
			var pfcomp = map.GetComponent<PathFinderMapComponent>();

			if (pfcomp == null)
			{
				pfcomp = new PathFinderMapComponent(map);
				map.components.Add(pfcomp);
			}
			return pfcomp.PathFinder.FindPath(start, dest, traverseParms, peMode);
		}
	}

	class PathFinderMapComponent : MapComponent
	{
		public readonly NewPathFinder PathFinder;

		public PathFinderMapComponent(Map map) : base(map)
		{
			PathFinder = new NewPathFinder(map);
		}
	}
#endregion

	public class NewPathFinder
	{
		private struct PathFinderNodeFast
		{
			public int knownCost;

			public int originalHeuristicCost;

			public int heuristicCost;

			public short perceivedPathCost;

			public ushort parentX;

			public ushort parentZ;

			public ushort status;
			
#if DEBUG
			public ushort timesPopped;
#endif
		}

		private struct PathFinderNode
		{
			public IntVec3 position;

			public IntVec3 parentPosition;
		}

		private class PathFinderNodeFastCostComparer : IComparer<CostNode>
		{
			private PathFinderNodeFast[] grid;

			public PathFinderNodeFastCostComparer(PathFinderNodeFast[] grid)
			{
				this.grid = grid;
			}

			public int Compare(CostNode a, CostNode b)
			{
				if (a.totalCostEstimate > b.totalCostEstimate) {
					return 1;
				}
				if (a.totalCostEstimate < b.totalCostEstimate) {
					return -1;
				}
				//The BPMX heuristic propagation results in a large number of ties. Using the pre-propagation
				//heuristic value works very well as a tie breaker
				var aCost = grid[a.gridIndex].originalHeuristicCost + grid[a.gridIndex].knownCost;
				var bCost = grid[b.gridIndex].originalHeuristicCost + grid[b.gridIndex].knownCost;
				if (aCost > bCost) { return 1; }
				if (aCost < bCost) { return -1; }
				return 0;
			}
		}


		internal struct CostNode
		{
			public CostNode(int index, int cost)
			{
				gridIndex = index;
				totalCostEstimate = cost;
			}

			public readonly int totalCostEstimate;

			public readonly int gridIndex;
		}

		public struct PawnPathCostSettings
		{
			public int moveTicksCardinal;

			public int moveTicksDiagonal;

			public ByteGrid avoidGrid;

			public Area area;
		}
		#region member variables
		private Map map;

		private BpmxFastPriortyQueue openList;

		private PathFinderNodeFast[] calcGrid;

		private ushort statusOpenValue = 1;

		private ushort statusClosedValue = 2;

		private readonly int mapSizeX;

		private readonly int mapSizeZ;

		private PathGrid pathGrid;

		private int[] pathGridDirect;

		private Building[] edificeGrid;

		private PawnPath newPath;

		private int moveTicksCardinal;

		private int moveTicksDiagonal;

		private int curIndex;

		private ushort curX;

		private ushort curZ;

		private IntVec3 curIntVec3 = default(IntVec3);

		private int neighIndex;

		private ushort neighX;

		private ushort neighZ;

		private int neighCostThroughCur;

		private int h;

		private int closedCellCount;

		private int destinationIndex;

		private int destinationX = -1;

		private int destinationZ = -1;

		private CellRect destinationRect;

		private bool destinationIsOneCell;

		private int heuristicStrength;

		private bool debug_pathFailMessaged;

		private int debug_totalOpenListCount;

		private int debug_openCellsPopped;
		
		private int debug_closedCellsReopened;

		private int debug_totalHeuristicCostEstimate;

		private readonly int[] neighIndexes = { -1, -1, -1, -1, -1, -1, -1, -1 };

		private SimpleCurve regionHeuristicWeight = null;
		#endregion
		
		//With a flat weight on the heurestic, it ends up trying very hard near the end of the path, while ignoring easy opportunities early on in the path
		//Weighting it on a curve lets us spread more of the effort across the whole path, getting the easy gains whereever they are along the path.
		private readonly SimpleCurve regionHeuristicWeightReal = new SimpleCurve
		{	//x values get adjusted each run
			new CurvePoint(0, 1.05f),
			new CurvePoint(2, 1.1f),
			new CurvePoint(4, 1.2f)
		};

		private readonly SimpleCurve regionHeuristicWeightNone = new SimpleCurve
		{	//x values get adjusted each run
			new CurvePoint(0, 1.0f),
			new CurvePoint(2, 1.0f),
			new CurvePoint(4, 1.0f)
		};


		//From testing 1.1 seems to be about the optimal value for this; raising it any higher increases cells opened
		internal static float diagonalPercievedCostWeight = 1.1f;

		private static readonly sbyte[] Directions = { 0, 1, 0, -1, 1, 1, -1, -1, -1, 0, 1, 0, -1, 1, 1, -1 };

		private static readonly SimpleCurve HeuristicStrengthHuman_DistanceCurve = new SimpleCurve
		{
			new CurvePoint(40f, 10f),
			new CurvePoint(130f, 35f)
		};

		private static bool weightEnabled = true;
		//Pathmax disabled because after a number of heuristic improvements it started making paths
		//worse/more expensive more often than it helped.
		private static bool pathmaxEnabled = false;
		

		internal static bool disableDebugFlash = false;

		public NewPathFinder(Map map)
		{
			this.map = map;
			var mapSizePowTwo = map.info.PowerOfTwoOverMapSize;
			var gridSizeX = (ushort)mapSizePowTwo;
			var gridSizeZ = (ushort)mapSizePowTwo;
			mapSizeX = map.Size.x;
			mapSizeZ = map.Size.z;
			calcGrid = new PathFinderNodeFast[gridSizeX * gridSizeZ];
			openList = new BpmxFastPriortyQueue(new PathFinderNodeFastCostComparer(calcGrid), gridSizeX * gridSizeZ);
		}



		internal enum HeuristicMode
		{
			Vanilla,
			AdmissableOctile,
			Better
		}


		
		public PawnPath FindPath(IntVec3 start, LocalTargetInfo dest, TraverseParms traverseParms, PathEndMode peMode = PathEndMode.OnCell)
		{
#if DEBUG

			if (traverseParms.pawn != null)
			{
				Log.Message($"Pathfinding times for pawn {traverseParms.pawn}, mode: {traverseParms.mode}\n Move speed: {traverseParms.pawn.TicksPerMoveCardinal}, {traverseParms.pawn.TicksPerMoveDiagonal}");
			}
			disableDebugFlash = true; //disable debug flash during timing tests
			var sw = new Stopwatch();
			PawnPath temp = null;
		    var vanillaCost = float.MaxValue - 1000;

			sw.Start();
			temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Vanilla);
			sw.Stop();
			Log.Message("~~ Vanilla ~~ " /*+ sw.ElapsedTicks + " ticks, "*/ + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!");
			vanillaCost = temp.TotalCost;
			temp.Dispose();

			//sw.Reset();
			//sw.Start();
			//temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.AdmissableOctile);
			//sw.Stop();
			//Log.Message("~~ Admissable Octile ~~ " + sw.ElapsedTicks + " ticks, " + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!");
			//var optimal = temp.TotalCost;
			//temp.Dispose();


			sw.Reset();
			sw.Start();
			temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Better);
			sw.Stop();
			Log.Message("~~ Better ~~ " /*+ sw.ElapsedTicks + " ticks, "*/ + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!  (" /*+ sw.ElapsedMilliseconds*/ + "ms)");

			//var sb = new StringBuilder();
			//foreach (var pathmax in new [] {false, true})
			//{
			//	pathmaxEnabled = pathmax;
			//	foreach (var weight in new[] {false, true})
			//	{
			//		weightEnabled = weight;
			//		sw.Reset();
			//		sw.Start();
			//		temp = FindPathInner(start, dest, traverseParms, peMode);
			//		sw.Stop();
			//		sb.AppendLine($"pathmax: {pathmax}, weight: {weight}, pops: {debug_openCellsPopped}, pathcost: {temp.TotalCost}, elapsed: {sw.ElapsedTicks}");
			//		temp.Dispose();
			//	}
			//}
			//Log.Message(sb.ToString());

			//Log.Message("\t Distance Map Pops: " + RegionLinkDijkstra.nodes_popped);
			//Log.Message("\t Total open cells added: " + debug_totalOpenListCount);
			//Log.Message("\t Closed cells reopened: " + debug_closedCellsReopened);
			//Log.Message($"\t Total Heuristic Estimate {debug_totalHeuristicCostEstimate}, off by {((temp.TotalCost / debug_totalHeuristicCostEstimate) - 1.0f).ToStringPercent()}");

			temp?.Dispose();
			disableDebugFlash = false;
#endif
#if PFPROFILE
			if (!hasRunOnce)
			{
				disableDebugFlash = true;
				var jitRun = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Better);
				jitRun.Dispose();
				hasRunOnce = true;
				disableDebugFlash = false;
			}
			sws.Clear();
#endif
			var result = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Better);

#if PFPROFILE
			var profsb = new StringBuilder();
			foreach (var pfsw in sws)
			{
				profsb.AppendLine("SW " + pfsw.Key + ": " + pfsw.Value.ElapsedTicks.ToString("##,#") + " ticks.");
			}
			Log.Message(profsb.ToString());
#endif
#if DEBUG
			if (Current.ProgramState == ProgramState.Playing)
			{
                if (debug_openCellsPopped > 5000 || (vanillaCost + 100) < result.TotalCost)
                {
                    PathDataLog.SaveFromPathCall(this.map, start, dest, traverseParms, peMode);
                }
            }
#endif
			return result;
		}

		internal static string FindPathTunableHeader(IEnumerable<float> tunableValues)
		{
			string header = "";
			foreach (var value in tunableValues) { header += $"|{value} pops|{value} cost"; }
			return header;
		}

		internal string FindPathTunableTest(IntVec3 start, LocalTargetInfo dest, TraverseParms traverseParms, PathEndMode peMode, IEnumerable<float> tunableValues)
		{
			string results = "";
			foreach (var value in tunableValues)
			{
				diagonalPercievedCostWeight = value;
				var path = this.FindPathInner(start, dest, traverseParms, peMode);
				results += $"|{debug_openCellsPopped}|{path.TotalCost}";
				path.Dispose();
			}
			return results;
		}


		private bool ValidateFindPathParameters(Pawn pawn, IntVec3 start, LocalTargetInfo dest, TraverseParms traverseParms, PathEndMode peMode, bool canPassAnything)
		{
			if (pawn != null && pawn.Map != this.map)
			{
				Log.Error(string.Concat("Tried to FindPath for pawn which is spawned in another map. His map PathFinder should have been used, not this one. pawn=", pawn, " pawn.Map=", pawn.Map, " map=", this.map));
				return false;
			}

			if (!start.IsValid)
			{
				Log.Error(string.Concat("Tried to FindPath with invalid start ", start, ", pawn= ", pawn));
				return false;
			}
			if (!dest.IsValid)
			{
				Log.Error(string.Concat("Tried to FindPath with invalid dest ", dest, ", pawn= ", pawn));
				return false;
			}

			if (!canPassAnything)
			{
                //For offline testing, reachability check can crash with null pawn
                if (Current.ProgramState == ProgramState.Playing)
                {
                    if (!this.map.reachability.CanReach(start, dest, peMode, traverseParms))
                    {
                        return false;
                    }
                }
				map.regionAndRoomUpdater.RebuildDirtyRegionsAndRooms();
			}
			else if (dest.HasThing && dest.Thing.Map != this.map)
			{
				return false;
			}
			return true;
		}
		


		//The standard A* search algorithm has been modified to implement the bidirectional pathmax algorithm
		//("Inconsistent heuristics in theory and practice" Felner et al.) http://web.cs.du.edu/~sturtevant/papers/incnew.pdf
		internal PawnPath FindPathInner(IntVec3 start, LocalTargetInfo dest, TraverseParms traverseParms, PathEndMode peMode, HeuristicMode mode = HeuristicMode.Better)
		{
			//The initialization is largely unchanged from Core, aside from coding style in some spots
#region initialization
			if (DebugSettings.pathThroughWalls) {
				traverseParms.mode = TraverseMode.PassAnything;
			}
			Pawn pawn = traverseParms.pawn;
			bool canPassAnything = traverseParms.mode == TraverseMode.PassAnything;

			if (!ValidateFindPathParameters(pawn, start, dest, traverseParms, peMode, canPassAnything))
			{
				return PawnPath.NotFound;
			}
			
			PfProfilerBeginSample(string.Concat("FindPath for ", pawn, " from ", start, " to ", dest, (!dest.HasThing) ? string.Empty : (" at " + dest.Cell)));
			destinationX = dest.Cell.x;
			destinationZ = dest.Cell.z;
			var cellIndices = this.map.cellIndices;
			curIndex = cellIndices.CellToIndex(start);
			destinationIndex = cellIndices.CellToIndex(dest.Cell);
			if (!dest.HasThing || peMode == PathEndMode.OnCell) {
				destinationRect = CellRect.SingleCell(dest.Cell);
			}
			else {
				destinationRect = dest.Thing.OccupiedRect();
			}
			if (peMode == PathEndMode.Touch) {
				destinationRect = destinationRect.ExpandedBy(1);
			}
			var regions = destinationRect.Cells.Where(c => c.InBounds(map)).Select(c => this.map.regionGrid.GetRegionAt_InvalidAllowed(c)).Where(r => r != null);
			//Pretty sure this shouldn't be able to happen...
			if (mode == HeuristicMode.Better && !canPassAnything && !regions.Any())
			{
				mode = HeuristicMode.Vanilla;
				Log.Warning("Pathfinding destination not in region, must fall back to vanilla!");
			}
			destinationIsOneCell = (destinationRect.Width == 1 && destinationRect.Height == 1);
			pathGrid = this.map.pathGrid;
			pathGridDirect = this.map.pathGrid.pathGrid;
			this.edificeGrid = this.map.edificeGrid.InnerArray;
			statusOpenValue += 2;
			statusClosedValue += 2;
			if (statusClosedValue >= 65435) {
				ResetStatuses();
			}
			if (pawn?.RaceProps.Animal == true) {
				heuristicStrength = 30;
			}
			else {
				float lengthHorizontal = (start - dest.Cell).LengthHorizontal;
				heuristicStrength = (int)Math.Round(HeuristicStrengthHuman_DistanceCurve.Evaluate(lengthHorizontal));
			}
			closedCellCount = 0;
			openList.Clear();
			debug_pathFailMessaged = false;
			debug_totalOpenListCount = 0;
			debug_openCellsPopped = 0;

			PawnPathCostSettings pawnPathCosts = GetPawnPathCostSettings(traverseParms.pawn);

			moveTicksCardinal = pawnPathCosts.moveTicksCardinal;
			moveTicksDiagonal = pawnPathCosts.moveTicksDiagonal;
			var diagonalAddedTicks = moveTicksDiagonal - moveTicksCardinal;

			//Where the magic happens
			RegionPathCostHeuristic regionCost = new RegionPathCostHeuristic(map, start, destinationRect, regions, traverseParms, pawnPathCosts);

			if (mode == HeuristicMode.Better)
			{
				if (canPassAnything)
				{
					//Roughly preserves the Vanilla behavior of increasing path accuracy for shorter paths and slower pawns, though not as smoothly. Only applies to sappers.
					heuristicStrength = Math.Max(1, (int) Math.Round(heuristicStrength / (float) moveTicksCardinal));
				}
				else
				{
					var totalCostEst = (debug_totalHeuristicCostEstimate = regionCost.GetPathCostToRegion(curIndex)) + (moveTicksCardinal * 50); //Add constant cost so it tries harder on short paths
					regionHeuristicWeightReal[1].x = totalCostEst / 2;
					regionHeuristicWeightReal[2].x = totalCostEst;
				}
				regionHeuristicWeight = weightEnabled ? regionHeuristicWeightReal : regionHeuristicWeightNone;
			}
			else
			{
				regionHeuristicWeight = regionHeuristicWeightNone;
			}
			calcGrid[curIndex].knownCost = 0;
			calcGrid[curIndex].heuristicCost = 0;
			calcGrid[curIndex].parentX = (ushort)start.x;
			calcGrid[curIndex].parentZ = (ushort)start.z;
			calcGrid[curIndex].status = statusOpenValue;
			openList.Push(new CostNode(curIndex, 0));
			

			bool shouldCollideWithPawns = false;
			if (pawn != null) {
				shouldCollideWithPawns = PawnUtility.ShouldCollideWithPawns(pawn);
			}
			#endregion

			while (true) {
				PfProfilerBeginSample("Open cell pop");
				if (openList.Count <= 0) {
					break;
				}
				debug_openCellsPopped++;
				var thisNode = openList.Pop();
				curIndex = thisNode.gridIndex;
				PfProfilerEndSample();
				PfProfilerBeginSample("Open cell");
				if (calcGrid[curIndex].status == statusClosedValue) {
					PfProfilerEndSample();
				}
				else
				{
#if DEBUG
					calcGrid[curIndex].timesPopped++;
#endif
					curIntVec3 = cellIndices.IndexToCell(curIndex);
					curX = (ushort)curIntVec3.x;
					curZ = (ushort)curIntVec3.z;
					if (DebugViewSettings.drawPaths && !disableDebugFlash && debug_openCellsPopped < 20000)
					{
						//draw backpointer
						var arrow = GetBackPointerArrow(calcGrid[curIndex].parentX, calcGrid[curIndex].parentZ, curX, curZ);
						string leading = "";
						string trailing = "";

#if DEBUG
						switch (calcGrid[curIndex].timesPopped)
						{
							case 1:
								trailing = "\n\n"; // $"\n\n\n{thisNode.totalCostEstimate}({calcGrid[curIndex].knownCost + calcGrid[curIndex].originalHeuristicCost})"; 
								break;
							case 2:
								trailing = "\n"; break;
							case 3: break;
							case 4:
								leading = "\n"; break;
							default:
								leading = "\n\n"; break;
						}
#endif
						DebugFlash(curIntVec3, calcGrid[curIndex].knownCost / 1500f, leading + calcGrid[curIndex].knownCost + " " + arrow + " " + debug_openCellsPopped + trailing);
					}
					if ((destinationIsOneCell && curIndex == destinationIndex) || destinationRect.Contains(curIntVec3))
					{
						PfProfilerEndSample();
						PfProfilerBeginSample("Finalize Path");
						var ret = FinalizedPath();
						PfProfilerEndSample();
						return ret;
					}
					//With reopening closed nodes, this limit can be reached a lot more easily. I've left it as is because it gets users to report bad paths. 
					if (closedCellCount > 160000) {
						Log.Warning(string.Concat(pawn, " pathing from ", start, " to ", dest, " hit search limit of ", 160000, " cells."));
						PfProfilerEndSample();
						return PawnPath.NotFound;
					}
					PfProfilerEndSample();
					PfProfilerBeginSample("Neighbor consideration");
					for (int i = 0; i < 8; i++)
					{
						neighIndexes[i] = -1;

                        neighX = (ushort)(curX + Directions[i]);
                        neighZ = (ushort)(curZ + Directions[i + 8]);
                        if (neighX >= mapSizeX || neighZ >= mapSizeZ) { continue; }

                        switch (i)
                        {
                            case 4:
                                if (!pathGrid.WalkableFast(curX, curZ - 1) || !pathGrid.WalkableFast(curX + 1, curZ)) { continue; }
                                break;
                            case 5:
                                if (!pathGrid.WalkableFast(curX, curZ + 1) || !pathGrid.WalkableFast(curX + 1, curZ)) { continue; }
                                break;
                            case 6:
                                if (!pathGrid.WalkableFast(curX, curZ + 1) || !pathGrid.WalkableFast(curX - 1, curZ)) { continue; }
                                break;
                            case 7:
                                if (!pathGrid.WalkableFast(curX, curZ - 1) || !pathGrid.WalkableFast(curX - 1, curZ)) { continue; }
                                break;
                        }

						neighIndex = cellIndices.CellToIndex(neighX, neighZ);

						if ((calcGrid[neighIndex].status != statusClosedValue) && (calcGrid[neighIndex].status != statusOpenValue))
						{
						    if (10000 <= (calcGrid[neighIndex].perceivedPathCost = GetTotalPerceivedPathCost(traverseParms, canPassAnything, shouldCollideWithPawns, pawn, pawnPathCosts)))
						    {
                                continue;
						    }
#if DEBUG
							calcGrid[neighIndex].timesPopped = 0;
#endif
							#region heuristic
							PfProfilerBeginSample("Heuristic");
							switch (mode)
							{
								case HeuristicMode.Vanilla:
									h = heuristicStrength * (Math.Abs(neighX - destinationX) + Math.Abs(neighZ - destinationZ));
									break;
								case HeuristicMode.AdmissableOctile:
									{
										var dx = Math.Abs(neighX - destinationX);
										var dy = Math.Abs(neighZ - destinationZ);
										h = moveTicksCardinal * (dx + dy) + (moveTicksDiagonal - 2 * moveTicksCardinal) * Math.Min(dx, dy);
									}
									break;
								case HeuristicMode.Better:
									if (canPassAnything)
									{
										var dx = Math.Abs(neighX - destinationX);
										var dy = Math.Abs(neighZ - destinationZ);
										h = heuristicStrength * (moveTicksCardinal * (dx + dy) + (moveTicksDiagonal - 2 * moveTicksCardinal) * Math.Min(dx, dy));
									}
									else
									{
										h = regionCost.GetPathCostToRegion(neighIndex);
									}
									break;
							}
							calcGrid[neighIndex].heuristicCost = h;
							calcGrid[neighIndex].originalHeuristicCost = h;
							PfProfilerEndSample();
							#endregion
						}


						if (calcGrid[neighIndex].perceivedPathCost < 10000)
						{
							neighIndexes[i] = neighIndex;
						}
						if ((calcGrid[neighIndex].status == statusOpenValue && 
							Math.Max(i > 3 ? (int)(calcGrid[curIndex].perceivedPathCost * diagonalPercievedCostWeight) + moveTicksDiagonal : calcGrid[curIndex].perceivedPathCost + moveTicksCardinal, 1) + calcGrid[neighIndex].knownCost < calcGrid[curIndex].knownCost))
						{
							calcGrid[curIndex].parentX = neighX;
							calcGrid[curIndex].parentZ = neighZ;
							calcGrid[curIndex].knownCost = Math.Max(i > 3 ? (int)(calcGrid[curIndex].perceivedPathCost * diagonalPercievedCostWeight) + moveTicksDiagonal : calcGrid[curIndex].perceivedPathCost + moveTicksCardinal, 1) + calcGrid[neighIndex].knownCost;
						}

					}
					#region BPMX Best H
					PfProfilerBeginSample("BPMX Best H");
					int bestH = calcGrid[curIndex].heuristicCost;
					if (mode == HeuristicMode.Better && pathmaxEnabled)
					{
						for (int i = 0; i < 8; i++)
						{
							neighIndex = neighIndexes[i];
							if (neighIndex < 0)
							{
								continue;
							}
							bestH = Math.Max(bestH, calcGrid[neighIndex].heuristicCost - (calcGrid[curIndex].perceivedPathCost + (i > 3 ? moveTicksDiagonal : moveTicksCardinal)));
						}
					}

					//Pathmax Rule 3: set the current node heuristic to the best value of all connected nodes
					calcGrid[curIndex].heuristicCost = bestH;
					PfProfilerEndSample();
					#endregion

					#region Updating open list
					for (int i = 0; i < 8; i++)
					{
						neighIndex = neighIndexes[i];
						if (neighIndex < 0) { continue; }
						if (calcGrid[neighIndex].status == statusClosedValue && (canPassAnything || mode != HeuristicMode.Better))
						{
							continue;
						}
						
                        //When path costs are significantly higher than move costs (e.g. snowy ice, or outside of allowed areas), 
						//small differences in the weighted heuristic overwhelm the added cost of diagonal movement, so nodes
						//can often be visited in unecessary zig-zags, causing lots of nodes to be reopened later, and weird looking
						//paths if they are not revisited. Weighting the diagonal path cost slightly counteracts this behavior, and
						//should result in natural looking paths when it does cause suboptimal behavior
						var thisDirEdgeCost = (i > 3 ? (int)(calcGrid[neighIndex].perceivedPathCost * diagonalPercievedCostWeight) + moveTicksDiagonal : calcGrid[neighIndex].perceivedPathCost + moveTicksCardinal);

						//var thisDirEdgeCost = calcGrid[neighIndex].perceivedPathCost + (i > 3 ? moveTicksDiagonal : moveTicksCardinal);
						//Some mods can result in negative path costs. That'll work well enough with Vanilla, since it won't revisit closed nodes, but when we do, it's an infinite loop.
						thisDirEdgeCost = (ushort)Math.Max(thisDirEdgeCost, 1);
						neighCostThroughCur = thisDirEdgeCost + calcGrid[curIndex].knownCost;
						//Pathmax Rule 1
						int nodeH = (mode == HeuristicMode.Better && pathmaxEnabled) ? Math.Max(calcGrid[neighIndex].heuristicCost, bestH - thisDirEdgeCost) : calcGrid[neighIndex].heuristicCost;

						if (calcGrid[neighIndex].status == statusClosedValue || calcGrid[neighIndex].status == statusOpenValue)
						{
							bool needsUpdate = false;
							int minReopenGain = 0;
							if (calcGrid[neighIndex].status == statusOpenValue)
							{
								needsUpdate = nodeH > calcGrid[neighIndex].heuristicCost;
							}
							else
							{	//Don't reopen closed nodes if the path cost difference isn't large enough to justify it; otherwise there can be cascades of revisiting the same nodes over and over for tiny path improvements each time
								minReopenGain = moveTicksCardinal;
								if (pawnPathCosts.area?[neighIndex] == false) { minReopenGain *= 10; }
							}
							calcGrid[neighIndex].heuristicCost = nodeH;
							
							if (!(neighCostThroughCur + minReopenGain < calcGrid[neighIndex].knownCost))
							{
								if (needsUpdate) //if the heuristic cost was increased for an open node, we need to adjust its spot in the queue
								{
								    var neighCell = cellIndices.IndexToCell(neighIndex);
                                    var edgeCost = Math.Max(calcGrid[neighIndex].parentX != neighCell.x && calcGrid[neighIndex].parentZ != neighCell.z ? (int)(calcGrid[neighIndex].perceivedPathCost * diagonalPercievedCostWeight) + moveTicksDiagonal : calcGrid[neighIndex].perceivedPathCost + moveTicksCardinal, 1);
									openList.PushOrUpdate(new CostNode(neighIndex, calcGrid[neighIndex].knownCost - edgeCost
																					 + (int)Math.Ceiling((edgeCost + nodeH) * regionHeuristicWeight.Evaluate(calcGrid[neighIndex].knownCost))));
								}
								continue;
							}
						}
						
#if DEBUG
						if (calcGrid[neighIndex].status == statusClosedValue)
						{
							debug_closedCellsReopened++;
						}
                        //else if (calcGrid[neighIndex].status != statusOpenValue)
                        //{
                        //    DebugFlash(cellIndices.IndexToCell(neighIndex), 0.2f, $"\n\n{calcGrid[neighIndex].originalHeuristicCost /*+ neighCostThroughCur*/} | {nodeH}\n" + $"{calcGrid[curIndex].knownCost + (int)Math.Ceiling((nodeH + thisDirEdgeCost) * regionHeuristicWeight.Evaluate(calcGrid[curIndex].knownCost))}");
                        //}
#endif

                        calcGrid[neighIndex].parentX = curX;
						calcGrid[neighIndex].parentZ = curZ;
						
						calcGrid[neighIndex].knownCost = neighCostThroughCur;
						calcGrid[neighIndex].status = statusOpenValue;
						calcGrid[neighIndex].heuristicCost = nodeH;

						PfProfilerBeginSample("Push Open");
						openList.PushOrUpdate(new CostNode(neighIndex, calcGrid[curIndex].knownCost
																		+ (int)Math.Ceiling((nodeH + thisDirEdgeCost)  * regionHeuristicWeight.Evaluate(calcGrid[curIndex].knownCost))));
						debug_totalOpenListCount++;
						PfProfilerEndSample();
					}
					#endregion
					PfProfilerEndSample();
					closedCellCount++;
					calcGrid[curIndex].status = statusClosedValue;
				}
			}
			if (!debug_pathFailMessaged) {
				string text = pawn?.CurJob?.ToString() ?? "null";
				string text2 = pawn?.Faction?.ToString() ?? "null";
				Log.Warning(string.Concat(pawn, " pathing from ", start, " to ", dest, " ran out of cells to process.\nJob:", text, "\nFaction: ", text2));
				debug_pathFailMessaged = true;
			}
			PfProfilerEndSample();
			return PawnPath.NotFound;
		}

	    private short GetTotalPerceivedPathCost(TraverseParms traverseParms, bool canPassAnything, bool shouldCollideWithPawns, Pawn pawn, PawnPathCostSettings pawnPathCosts)
	    {
	        int neighCost = 0;
		    if (!pathGrid.WalkableFast(neighX, neighZ))
		    {
			    if (!canPassAnything)
			    {
				    return 10000;
			    }
			    neighCost += 60;
			    Thing edifice = edificeGrid[neighIndex];
			    if (edifice == null || !edifice.def.useHitPoints) { return 10000; }
			    neighCost += (int) (edifice.HitPoints * 0.1f);
		    }
		    else
		    {
			    neighCost += pathGridDirect[neighIndex];
		    }
	        if (shouldCollideWithPawns && PawnUtility.AnyPawnBlockingPathAt(new IntVec3(neighX, 0, neighZ), pawn))
	        {
	            neighCost += 800;
	        }
	        Building building = edificeGrid[neighIndex];
	        if (building != null)
	        {
	            int cost = GetPathCostForBuilding(building, traverseParms);
	            if (cost < 0) { return 10000; }
	            neighCost += cost;
	        }
	        if (pawnPathCosts.avoidGrid != null)
	        {
	            neighCost += pawnPathCosts.avoidGrid[neighIndex] * 8;
	        }
	        if (pawnPathCosts.area?[neighIndex] == false)
	        {   //Allowed area path cost switched from adding a constant to multiplying the path cost
				//Mostly this was to reduce reopens when very high path cost nodes get explored greedily (like trees)
				//But I would imagine the player would generally prefer their pawns not spend ages walking over several trees
				//just to get a path one tile shorter.
	            neighCost = (Math.Max(neighCost, 10) + moveTicksCardinal * 2) * 10;
	        }
	        return (short)Math.Min(neighCost, 9999);
	    }

	    public static Func<Pawn, PawnPathCostSettings> GetPawnPathCostSettings = GetPawnPathCostSettingsDefault;

		private static PawnPathCostSettings GetPawnPathCostSettingsDefault(Pawn pawn)
		{
			return new PawnPathCostSettings
			{
				moveTicksCardinal = pawn?.TicksPerMoveCardinal ?? 13,
				moveTicksDiagonal = pawn?.TicksPerMoveDiagonal ?? 18,
				avoidGrid = pawn?.GetAvoidGrid(),
				area = pawn?.playerSettings?.AreaRestrictionInPawnCurrentMap
			};
		}

		public static Func<Building, TraverseParms, int> GetPathCostForBuilding = GetPathCostForBuildingDefault;

		private static int GetPathCostForBuildingDefault(Building building, TraverseParms traverseParms)
		{
			Building_Door building_Door = building as Building_Door;
			if (building_Door != null)
			{
				switch (traverseParms.mode)
				{
					case TraverseMode.ByPawn:
						if (!traverseParms.canBash && building_Door.IsForbiddenToPass(traverseParms.pawn))
						{
							return -1;
						}
						if (!building_Door.FreePassage)
						{
							if (building_Door.PawnCanOpen(traverseParms.pawn)) { return building_Door.TicksToOpenNow; }
							return !traverseParms.canBash ? -1 : 300;
						}
						break;
					case TraverseMode.NoPassClosedDoors:
						if (!building_Door.FreePassage) { return -1; }
						break;
				}
			}
			else if (traverseParms.pawn != null)
			{
				return building.PathFindCostFor(traverseParms.pawn);
			}
			return 0;
		}

        private static char GetBackPointerArrow(IntVec3 prev, IntVec3 cur)
        {
            return GetBackPointerArrow(prev.x, prev.z, cur.x, cur.z);
        }


        private static char GetBackPointerArrow(int prevX, int prevZ, int curX, int curZ)
		{
			char arrow;
			if (prevX < curX)
			{
				if (prevZ < curZ) arrow = '↙';
				else if (prevZ > curZ) arrow = '↖';
				else arrow = '←';
			}
			else if (prevX > curX)
			{
				if (prevZ < curZ) arrow = '↘';
				else if (prevZ > curZ) arrow = '↗';
				else arrow = '→';
			}
			else
			{
				if (prevZ < curZ) arrow = '↓';
				else if (prevZ > curZ) arrow = '↑';
				else arrow = 'x'; //'⥁'; //unpossible (apparently RimWorld's font doesn't have the loop :( )
			}
			return arrow;
		}


		internal void DebugFlash(IntVec3 c, float colorPct, string str)
		{
			DebugFlash(this.map, c, colorPct, str);
		}

		internal static void DebugFlash(Map map, IntVec3 c, float colorPct, string str)
		{
			if (DebugViewSettings.drawPaths && !disableDebugFlash)
			{
				map.debugDrawer.FlashCell(c, colorPct, str);
			}
		}

		internal static void DebugLine(Map map, IntVec3 a, IntVec3 b)
		{
			if (DebugViewSettings.drawPaths && !disableDebugFlash)
			{
				map.debugDrawer.FlashLine(a, b);
			}
		}

		private PawnPath FinalizedPath()
		{
			newPath = this.map.pawnPathPool.GetEmptyPawnPath();
			IntVec3 parentPosition = new IntVec3(curX, 0, curZ); 
			var cellIndices = this.map.cellIndices;
			int prevKnownCost = calcGrid[cellIndices.CellToIndex(parentPosition)].knownCost;
			int actualCost = 0;
			while (true) {
				PathFinderNodeFast pathFinderNodeFast = calcGrid[cellIndices.CellToIndex(parentPosition)];
				PathFinderNode pathFinderNode;
				pathFinderNode.parentPosition = new IntVec3(pathFinderNodeFast.parentX, 0, pathFinderNodeFast.parentZ);
				pathFinderNode.position = parentPosition;
				newPath.AddNode(pathFinderNode.position);

				actualCost += prevKnownCost - pathFinderNodeFast.knownCost;
				prevKnownCost = pathFinderNodeFast.knownCost;
				var hDiscrepancy = actualCost - pathFinderNodeFast.heuristicCost;
				DebugFlash(parentPosition, hDiscrepancy / 100f, "\n\n" /*+ actualCost + "\n"*/ + hDiscrepancy);
				if (pathFinderNode.position == pathFinderNode.parentPosition) {
					break;
				}
				parentPosition = pathFinderNode.parentPosition;
			}
			newPath.SetupFound(calcGrid[curIndex].knownCost);
			PfProfilerEndSample();
			return newPath;
		}

		private void ResetStatuses()
		{
			int num = calcGrid.Length;
			for (int i = 0; i < num; i++) {
				calcGrid[i].status = 0;
			}
			statusOpenValue = 1;
			statusClosedValue = 2;
		}


	#if PFPROFILE

		private static Dictionary<string, Stopwatch> sws = new Dictionary<string, Stopwatch>();

		private static Stack<Stopwatch> currSw = new Stack<Stopwatch>();

		private static bool hasRunOnce;

	#endif

		[Conditional("PFPROFILE")]
		public static void PfProfilerBeginSample(string s)
		{
	#if PFPROFILE
			Stopwatch sw;
			if (!sws.TryGetValue(s, out sw))
			{
				sw = sws[s] = new Stopwatch();
			}
			currSw.Push(sw);
			sw.Start();
	#endif
		}

		[Conditional("PFPROFILE")]
		public static void PfProfilerEndSample()
		{
#if PFPROFILE
			currSw.Pop()?.Stop();
#endif
		}

	}
}
