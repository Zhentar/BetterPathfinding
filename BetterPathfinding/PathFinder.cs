using System;
using System.Collections.Generic;
using System.Diagnostics;
using RimWorld;
using UnityEngine;
using Verse;
using Verse.AI;

namespace BetterPathfinding
{
	static class PathFinder
	{
		internal struct PathFinderNodeFast
		{
			public int knownCost;

			public int heuristicCost;

			public ushort parentX;

			public ushort parentZ;

			public ushort status;
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
				return 0;
			}
		}

		private struct CostNode
		{
			public CostNode(int index, int cost)
			{
				gridIndex = index;
				totalCostEstimate = cost;
			}

			public int totalCostEstimate;

			public int gridIndex;
		}

		private static FastPriorityQueue<CostNode> openList;

		private static PathFinderNodeFast[] calcGrid;

		private static ushort statusOpenValue = 1;

		private static ushort statusClosedValue = 2;

		private static int mapSizePowTwo;

		private static ushort gridSizeX;

		private static ushort gridSizeZ;

		private static ushort gridSizeXMinus1;

		private static ushort gridSizeZLog2;

		private static int mapSizeX;

		private static int mapSizeZ;

		private static PathGrid pathGrid;

		private static int[] pathGridDirect;

		private static Building[] edificeGrid;

		private static PawnPath newPath;

		private static int moveTicksCardinal;

		private static int moveTicksDiagonal;

		private static int curIndex;

		private static ushort curX;

		private static ushort curZ;

		private static IntVec3 curIntVec3 = default(IntVec3);

		private static int neighIndex;

		private static ushort neighX;

		private static ushort neighZ;

		private static int neighCostThroughCur;

		private static int neighCost;

		private static int h;

		private static int closedCellCount;

		private static int destinationIndex;

		private static int destinationX = -1;

		private static int destinationZ = -1;

		private static CellRect destinationRect;

		private static bool destinationIsOneCell;

		private static int heuristicStrength;

		private static bool debug_pathFailMessaged;

		private static int debug_totalOpenListCount;

		private static int debug_openCellsPopped;

		private static readonly sbyte[] Directions = {
			0,
			1,
			0,
			-1,
			1,
			1,
			-1,
			-1,
			-1,
			0,
			1,
			0,
			-1,
			1,
			1,
			-1
		};

		private static readonly SimpleCurve HeuristicStrengthHuman_DistanceCurve = new SimpleCurve
		{
			new CurvePoint(40f, 10f),
			new CurvePoint(130f, 35f)
		};

		private static bool disableDebugFlash = false;

		public static void _Reinit()
		{
			mapSizePowTwo = Find.Map.info.PowerOfTwoOverMapSize;
			gridSizeX = (ushort)mapSizePowTwo;
			gridSizeZ = (ushort)mapSizePowTwo;
			gridSizeXMinus1 = (ushort)(gridSizeX - 1);
			gridSizeZLog2 = (ushort)Math.Log(gridSizeZ, 2.0);
			mapSizeX = Find.Map.Size.x;
			mapSizeZ = Find.Map.Size.z;
			calcGrid = new PathFinderNodeFast[gridSizeX * gridSizeZ];
			openList = new FastPriorityQueue<CostNode>(new PathFinderNodeFastCostComparer(calcGrid));
		}

		private enum HeuristicMode
		{
			Vanilla,
			AdmissableOctile,
			Better
		}


		public static PawnPath _FindPath(IntVec3 start, TargetInfo dest, TraverseParms traverseParms, PathEndMode peMode = PathEndMode.OnCell)
		{
			var result = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Better);

#if DEBUG
			if (traverseParms.pawn != null)
			{
				Log.Message("Pathfinding times for pawn " + traverseParms.pawn.Name);
			}
			disableDebugFlash = true; //disable debug flash during timing tests
			var sw = new Stopwatch();

			sw.Start();
			var temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.AdmissableOctile);
			sw.Stop();
			Log.Message("~~ Admissable Octile ~~ " + sw.ElapsedTicks + " ticks, " + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!");
			temp.Dispose();

			sw.Reset();
			sw.Start();
			temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Vanilla);
			sw.Stop();
			Log.Message("~~ Vanilla ~~ " + sw.ElapsedTicks + " ticks, " + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!");
			temp.Dispose();
			sw.Reset();

			//re-run instead of timing the first run, to cut out jit overhead cost
			sw.Start();
			temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Better);
			sw.Stop();
			Log.Message("~~ Better ~~ " + sw.ElapsedTicks + " ticks, " + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!");
            if (RegionPathCostHeuristic.DijkstraStopWatch != null)
            {
                Log.Message("\t Distance Map Time: " + RegionPathCostHeuristic.DijkstraStopWatch.ElapsedTicks + " ticks.");
                Log.Message("\t Distance Map Pops: " + RegionLinkDijkstra.nodes_popped);
                RegionPathCostHeuristic.DijkstraStopWatch = null;
            }
			temp.Dispose();
			disableDebugFlash = false;
#endif

			return result;
		}

		private static PawnPath FindPathInner(IntVec3 start, TargetInfo dest, TraverseParms traverseParms, PathEndMode peMode, HeuristicMode mode = HeuristicMode.Better)
		{
			if (DebugSettings.pathThroughWalls) {
				traverseParms.mode = TraverseMode.PassAnything;
			}
			Pawn pawn = traverseParms.pawn;
			bool canPassAnything = traverseParms.mode == TraverseMode.PassAnything;
			if (!start.IsValid) {
				Log.Error(string.Concat("Tried to FindPath with invalid start ", start, ", pawn= ", pawn));
				return PawnPath.NotFound;
			}
			if (!dest.IsValid) {
				Log.Error(string.Concat("Tried to FindPath with invalid dest ", dest, ", pawn= ", pawn));
				return PawnPath.NotFound;
			}
			RegionPathCostHeuristic regionCost = null;

			if (!canPassAnything) {
				if (traverseParms.mode == TraverseMode.ByPawn) {
					if (!pawn.CanReach(dest, peMode, Danger.Deadly, traverseParms.canBash, traverseParms.mode)) {
						return PawnPath.NotFound;
					}
				}
				else if (!start.CanReach(dest, peMode, traverseParms)) {
					return PawnPath.NotFound;
				}
			}


			ByteGrid avoidGrid = pawn?.GetAvoidGrid();
			PfProfilerBeginSample(string.Concat("FindPath for ", pawn, " from ", start, " to ", dest, (!dest.HasThing) ? string.Empty : (" at " + dest.Cell)));
			destinationX = dest.Cell.x;
			destinationZ = dest.Cell.z;
			curIndex = CellIndices.CellToIndex(start);
			destinationIndex = CellIndices.CellToIndex(dest.Cell);
			if (!dest.HasThing || peMode == PathEndMode.OnCell) {
				destinationRect = CellRect.SingleCell(dest.Cell);
			}
			else {
				destinationRect = dest.Thing.OccupiedRect();
			}
			if (peMode == PathEndMode.Touch) {
				destinationRect = destinationRect.ExpandedBy(1);
			}
			if (mode == HeuristicMode.Better && Find.RegionGrid.GetValidRegionAt(dest.Cell) == null)
			{ //TODO: handle path end touch to no region tiles.
				mode = HeuristicMode.Vanilla;
			}
			destinationIsOneCell = (destinationRect.Width == 1 && destinationRect.Height == 1);
			pathGrid = Find.PathGrid;
			pathGridDirect = Find.PathGrid.pathGrid;
			edificeGrid = Find.EdificeGrid.InnerArray;
			statusOpenValue += 2;
			statusClosedValue += 2;
			if (statusClosedValue >= 65435) {
				ResetStatuses();
			}
			if (pawn != null && pawn.RaceProps.Animal) {
				heuristicStrength = 30;
			}
			else {
				float lengthHorizontal = (start - dest.Cell).LengthHorizontal;
				heuristicStrength = Mathf.RoundToInt(HeuristicStrengthHuman_DistanceCurve.Evaluate(lengthHorizontal));
			}
			closedCellCount = 0;
			openList.Clear();
			debug_pathFailMessaged = false;
			debug_totalOpenListCount = 0;
			debug_openCellsPopped = 0;
			if (pawn != null) {
				moveTicksCardinal = pawn.TicksPerMoveCardinal;
				moveTicksDiagonal = pawn.TicksPerMoveDiagonal;
			}
			else {
				moveTicksCardinal = 13;
				moveTicksDiagonal = 18;
			}

			if (mode == HeuristicMode.Better)
			{   //Roughly preserves the Vanilla behavior of increasing path accuracy for shorter paths and slower pawns, though not as smoothly
				heuristicStrength = Mathf.Max(1, Mathf.RoundToInt(heuristicStrength / (float)moveTicksCardinal));
			}

			regionCost = new RegionPathCostHeuristic(start, dest.Cell, traverseParms, moveTicksCardinal, moveTicksDiagonal);
			calcGrid[curIndex].knownCost = 0;
			calcGrid[curIndex].heuristicCost = 0;
			calcGrid[curIndex].parentX = (ushort)start.x;
			calcGrid[curIndex].parentZ = (ushort)start.z;
			calcGrid[curIndex].status = statusOpenValue;
			openList.Push(new CostNode(curIndex, 0));
			Area area = null;
			if (pawn != null && pawn.playerSettings != null && !pawn.Drafted) {
				area = pawn.playerSettings.AreaRestriction;
			}
			bool flag3 = false;
			if (pawn != null) {
				flag3 = PawnUtility.ShouldCollideWithPawns(pawn);
			}
			while (true) {
				PfProfilerBeginSample("Open cell");
				if (openList.Count <= 0) {
					break;
				}
				debug_totalOpenListCount += openList.Count;
				debug_openCellsPopped++;
				curIndex = openList.Pop().gridIndex;
				if (calcGrid[curIndex].status == statusClosedValue) {
					PfProfilerEndSample();
				}
				else {
					curIntVec3 = CellIndices.IndexToCell(curIndex);
					curX = (ushort)curIntVec3.x;
					curZ = (ushort)curIntVec3.z;
					if (DebugViewSettings.drawPaths && !disableDebugFlash)
					{
						//draw backpointer
						char arrow;
						int parX = calcGrid[curIndex].parentX;
						int parZ = calcGrid[curIndex].parentZ;
						if (parX < curX)
						{
							if (parZ < curZ) arrow = '↙';
							else if (parZ > curZ) arrow = '↖';
							else arrow = '←';
						}
						else if (parX > curX)
						{

							if (parZ < curZ) arrow = '↘';
							else if (parZ > curZ) arrow = '↗';
							else arrow = '→';
						}
						else
						{
							if (parZ < curZ) arrow = '↓';
							else if (parZ > curZ) arrow = '↑';
							else arrow = '⥁'; //unpossible
						}
						DebugFlash(curIntVec3, calcGrid[curIndex].knownCost / 1500f, calcGrid[curIndex].knownCost + " " + arrow);
					}
					if (destinationIsOneCell) {
						if (curIndex == destinationIndex) {
							PfProfilerEndSample();
							return FinalizedPath();
						}
					}
					else if (destinationRect.Contains(curIntVec3)) {
						PfProfilerEndSample();
						return FinalizedPath();
					}
					if (closedCellCount > 160000) {
						Log.Warning(string.Concat(pawn, " pathing from ", start, " to ", dest, " hit search limit of ", 160000, " cells."));
						DebugDrawRichData();
						PfProfilerEndSample();
						return PawnPath.NotFound;
					}
					PfProfilerEndSample();
					PfProfilerBeginSample("Neighbor consideration");
					for (int i = 0; i < 8; i++) {
						neighX = (ushort)(curX + Directions[i]);
						neighZ = (ushort)(curZ + Directions[i + 8]);
						IntVec3 intVec = new IntVec3(neighX, 0, neighZ);
						neighIndex = CellIndices.CellToIndex(neighX, neighZ);
						if (neighX >= mapSizeX || neighZ >= mapSizeZ) {
							DebugFlash(intVec, 0.75f, "oob");
						}
						else if (calcGrid[neighIndex].status != statusClosedValue || (mode == HeuristicMode.Better && !canPassAnything)) {
							int cost = 0;
							bool notWalkable = false;
							if (!pathGrid.WalkableFast(intVec)) {
								if (!canPassAnything) {
									DebugFlash(intVec, 0.22f, "walk");
									continue;
								}
								notWalkable = true;
								cost += 60;
								Thing edifice = intVec.GetEdifice();
								if (edifice == null || !edifice.def.useHitPoints)
								{
									continue;
								}
								cost += (int)(edifice.HitPoints * 0.1f);
							}
							if (i > 3) {
								switch (i) {
									case 4:
										if (!pathGrid.WalkableFast(curX, curZ - 1)) {
											continue;
										}
										if (!pathGrid.WalkableFast(curX + 1, curZ)) {
											continue;
										}
										break;
									case 5:
										if (!pathGrid.WalkableFast(curX, curZ + 1)) {
											continue;
										}
										if (!pathGrid.WalkableFast(curX + 1, curZ)) {
											continue;
										}
										break;
									case 6:
										if (!pathGrid.WalkableFast(curX, curZ + 1)) {
											continue;
										}
										if (!pathGrid.WalkableFast(curX - 1, curZ)) {
											continue;
										}
										break;
									case 7:
										if (!pathGrid.WalkableFast(curX, curZ - 1)) {
											continue;
										}
										if (!pathGrid.WalkableFast(curX - 1, curZ)) {
											continue;
										}
										break;
								}
							}
							neighCost = i > 3 ? moveTicksDiagonal : moveTicksCardinal;
							neighCost += cost;
							if (!notWalkable) {
								neighCost += pathGridDirect[neighIndex];
							}
							if (avoidGrid != null) {
								neighCost += avoidGrid[neighIndex] * 8;
							}
							if (area != null && !area[intVec]) {
								neighCost += 600;
							}
							if (flag3 && PawnUtility.AnyPawnBlockingPathAt(intVec, pawn)) {
								neighCost += 800;
							}
							Building building = edificeGrid[CellIndices.CellToIndex(neighX, neighZ)];
							if (building != null) {
								PfProfilerBeginSample("Edifices");
								Building_Door building_Door = building as Building_Door;
								if (building_Door != null) {
									switch (traverseParms.mode) {
										case TraverseMode.ByPawn:
											if (!traverseParms.canBash && building_Door.IsForbiddenToPass(pawn)) {
												if (DebugViewSettings.drawPaths) {
													DebugFlash(building.Position, 0.77f, "forbid");
												}
												PfProfilerEndSample();
												continue;
											}
											if (!building_Door.FreePassage) {
												if (building_Door.PawnCanOpen(pawn)) {
													neighCost += building_Door.TicksToOpenNow;
												}
												else {
													if (!traverseParms.canBash) {
														if (DebugViewSettings.drawPaths) {
															DebugFlash(building.Position, 0.34f, "cant pass");
														}
														PfProfilerEndSample();
														continue;
													}
													neighCost += 300;
												}
											}
											break;
										case TraverseMode.NoPassClosedDoors:
											if (!building_Door.FreePassage) {
												PfProfilerEndSample();
												continue;
											}
											break;
									}
								}
								else if (pawn != null) {
									neighCost += building.PathFindCostFor(pawn);
								}
								PfProfilerEndSample();
							}
							neighCostThroughCur = neighCost + calcGrid[curIndex].knownCost;

							if ((calcGrid[neighIndex].status != statusClosedValue && calcGrid[neighIndex].status != statusOpenValue) || calcGrid[neighIndex].knownCost > neighCostThroughCur)
							{
								calcGrid[neighIndex].parentX = curX;
								calcGrid[neighIndex].parentZ = curZ;

								if (calcGrid[neighIndex].status == statusClosedValue || calcGrid[neighIndex].status == statusOpenValue)
								{
									h = calcGrid[neighIndex].heuristicCost;
								}
								else
								{
									switch (mode)
									{
										case HeuristicMode.Vanilla:
											h = heuristicStrength*(Mathf.Abs(neighX - destinationX) + Mathf.Abs(neighZ - destinationZ));
											break;
										case HeuristicMode.AdmissableOctile:
										{
											var dx = Mathf.Abs(neighX - destinationX);
											var dy = Mathf.Abs(neighZ - destinationZ);
											h = moveTicksCardinal*(dx + dy) + (moveTicksDiagonal - 2*moveTicksCardinal)*Mathf.Min(dx, dy);
										}
											break;
										case HeuristicMode.Better:
											if (canPassAnything)
											{
												var dx = Mathf.Abs(neighX - destinationX);
												var dy = Mathf.Abs(neighZ - destinationZ);
												h = heuristicStrength * (moveTicksCardinal*(dx + dy) + (moveTicksDiagonal - 2*moveTicksCardinal)*Mathf.Min(dx, dy));
											}
											else
                                            {
                                                h = regionCost.GetPathCostToRegion(neighIndex);
												//DebugFlash(intVec, neighCostThroughCur + h / 1500f, (neighCostThroughCur + h).ToString());
											}
											break;
									}
								}

								calcGrid[neighIndex].knownCost = neighCostThroughCur;
								calcGrid[neighIndex].status = statusOpenValue;
								calcGrid[neighIndex].heuristicCost = h;

								//(Vanilla Fix) Always need to re-add, otherwise it won't get resorted into the right place
								openList.Push(new CostNode(neighIndex, neighCostThroughCur + h));
							}
						}
					}
					PfProfilerEndSample();
					closedCellCount++;
					calcGrid[curIndex].status = statusClosedValue;
				}
			}
			if (!debug_pathFailMessaged) {
				string text = pawn?.CurJob?.ToString() ?? "null";
				string text2 = pawn?.Faction?.ToString() ?? "null";
				Log.Warning(string.Concat(pawn, " pathing from ", start, " to ", dest, " ran out of cells to process.\nJob:", text, "\nFaction: ", text2, "\n\nThis will be the last message to avoid spam."));
				debug_pathFailMessaged = true;
			}
			DebugDrawRichData();
			PfProfilerEndSample();
			return PawnPath.NotFound;
		}


		internal static void DebugFlash(IntVec3 c, float colorPct, string str)
		{
			if (DebugViewSettings.drawPaths && !disableDebugFlash) {
				Find.DebugDrawer.FlashCell(c, colorPct, str);
			}
		}

		private static PawnPath FinalizedPath()
		{
			newPath = PawnPathPool.GetEmptyPawnPath();
			IntVec3 parentPosition = new IntVec3(curX, 0, curZ);
			int prevKnownCost = calcGrid[CellIndices.CellToIndex(parentPosition)].knownCost;
			int actualCost = 0;
			while (true) {
				PathFinderNodeFast pathFinderNodeFast = calcGrid[CellIndices.CellToIndex(parentPosition)];
				PathFinderNode pathFinderNode;
				pathFinderNode.parentPosition = new IntVec3(pathFinderNodeFast.parentX, 0, pathFinderNodeFast.parentZ);
				pathFinderNode.position = parentPosition;
				newPath.AddNode(pathFinderNode.position);

				//actualCost += prevKnownCost - pathFinderNodeFast.knownCost;
				//prevKnownCost = pathFinderNodeFast.knownCost;
				//var hDiscrepancy = actualCost - pathFinderNodeFast.heuristicCost;
				//DebugFlash(parentPosition, hDiscrepancy / 150f, hDiscrepancy + " (" + actualCost + "-" + pathFinderNodeFast.heuristicCost + ")");
				if (pathFinderNode.position == pathFinderNode.parentPosition) {
					break;
				}
				parentPosition = pathFinderNode.parentPosition;
			}
			newPath.SetupFound(calcGrid[curIndex].knownCost);
			PfProfilerEndSample();
			return newPath;
		}

		private static void ResetStatuses()
		{
			int num = calcGrid.Length;
			for (int i = 0; i < num; i++) {
				calcGrid[i].status = 0;
			}
			statusOpenValue = 1;
			statusClosedValue = 2;
		}

		[Conditional("PFPROFILE")]
		private static void PfProfilerBeginSample(string s)
		{
		}

		[Conditional("PFPROFILE")]
		private static void PfProfilerEndSample()
		{
		}

		private static void DebugDrawRichData()
		{
			if (DebugViewSettings.drawPaths) {
				while (openList.Count > 0) {
					int num = openList.Pop().gridIndex;
					IntVec3 c = new IntVec3(num & gridSizeXMinus1, 0, num >> gridSizeZLog2);
					Find.DebugDrawer.FlashCell(c, 0f, "open");
				}
			}
		}
	}
}
