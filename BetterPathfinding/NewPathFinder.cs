using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using RimWorld;
using UnityEngine;
using Verse;
using Verse.AI;

namespace BetterPathfinding
{

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

	class NewPathFinder
	{
		internal struct PathFinderNodeFast
		{
			public int knownCost;

			public int heuristicCost;

			public ushort edgeCost;

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

		private Map map;

		private BpmxFastPriortyQueue openList;

		private PathFinderNodeFast[] calcGrid;

		private ushort statusOpenValue = 1;

		private ushort statusClosedValue = 2;

		private int mapSizePowTwo;

		private ushort gridSizeX;

		private ushort gridSizeZ;

		private int mapSizeX;

		private int mapSizeZ;

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

		private int neighCost;

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

		private int[] neighIndexes = { -1, -1, -1, -1, -1, -1, -1, -1 };

		private SimpleCurve regionHeuristicWeight = new SimpleCurve
		{	//x values get adjusted 
			new CurvePoint(1, 1.25f),
			new CurvePoint(2, 1.12f),
			new CurvePoint(4, 1.05f)
		};

		private static readonly sbyte[] Directions = { 0, 1, 0, -1, 1, 1, -1, -1, -1, 0, 1, 0, -1, 1, 1, -1 };

		private static readonly SimpleCurve HeuristicStrengthHuman_DistanceCurve = new SimpleCurve
		{
			new CurvePoint(40f, 10f),
			new CurvePoint(130f, 35f)
		};

		internal static bool disableDebugFlash = false;

		public NewPathFinder(Map map)
		{
			this.map = map;
			mapSizePowTwo = map.info.PowerOfTwoOverMapSize;
			gridSizeX = (ushort)mapSizePowTwo;
			gridSizeZ = (ushort)mapSizePowTwo;
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
				Log.Message("Pathfinding times for pawn " + traverseParms.pawn.Name + ", mode: " + traverseParms.mode.ToString());
				Log.Message("Move speed: " + traverseParms.pawn.TicksPerMoveCardinal + "," + traverseParms.pawn.TicksPerMoveDiagonal);
			}
			disableDebugFlash = true; //disable debug flash during timing tests
			var sw = new Stopwatch();

			sw.Start();
			var temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Vanilla);
			sw.Stop();
			Log.Message("~~ Vanilla ~~ " + sw.ElapsedTicks + " ticks, " + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!");
			temp.Dispose();

			//sw.Reset();
			//sw.Start();
			//temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.AdmissableOctile);
			//sw.Stop();
			//Log.Message("~~ Admissable Octile ~~ " + sw.ElapsedTicks + " ticks, " + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!");
			//temp.Dispose();

			sw.Reset();
			sw.Start();
			temp = FindPathInner(start, dest, traverseParms, peMode, HeuristicMode.Better);
			sw.Stop();
			Log.Message("~~ Better ~~ " + sw.ElapsedTicks + " ticks, " + debug_openCellsPopped + " open cells popped, " + temp.TotalCost + " path cost!  (" + sw.ElapsedMilliseconds + "ms)");
            {
                Log.Message("\t Distance Map Pops: " + RegionLinkDijkstra.nodes_popped);
            }
		
			Log.Message("\t Total open cells added: " + debug_totalOpenListCount);
			Log.Message("\t Closed cells reopened: " + debug_closedCellsReopened);

			temp.Dispose();
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
			foreach (var pfsw in sws)
			{
				Log.Message("\t SW " + pfsw.Key + ": " + pfsw.Value.ElapsedTicks + " ticks.");
			}
#endif

			return result;
		}

		internal PawnPath FindPathInner(IntVec3 start, LocalTargetInfo dest, TraverseParms traverseParms, PathEndMode peMode, HeuristicMode mode = HeuristicMode.Better)
		{
			if (DebugSettings.pathThroughWalls) {
				traverseParms.mode = TraverseMode.PassAnything;
			}
			Pawn pawn = traverseParms.pawn;
			bool canPassAnything = traverseParms.mode == TraverseMode.PassAnything;

			if (pawn != null && pawn.Map != this.map)
			{
				Log.Error(string.Concat("Tried to FindPath for pawn which is spawned in another map. His map PathFinder should have been used, not this one. pawn=", pawn, " pawn.Map=", pawn.Map, " map=", this.map));
				return PawnPath.NotFound;
			}

			if (!start.IsValid) {
				Log.Error(string.Concat("Tried to FindPath with invalid start ", start, ", pawn= ", pawn));
				return PawnPath.NotFound;
			}
			if (!dest.IsValid) {
				Log.Error(string.Concat("Tried to FindPath with invalid dest ", dest, ", pawn= ", pawn));
				return PawnPath.NotFound;
			}

			if (!canPassAnything) {
				if (!this.map.reachability.CanReach(start, dest, peMode, traverseParms))
				{
					return PawnPath.NotFound;
				}
				map.regionAndRoomUpdater.RebuildDirtyRegionsAndRooms();
			}
			else if (dest.HasThing && dest.Thing.Map != this.map)
			{
				return PawnPath.NotFound;
			}


			ByteGrid avoidGrid = pawn?.GetAvoidGrid();
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
			var regions = destinationRect.Cells.Where(c => c.InBounds(map)).Select(c => this.map.regionGrid.GetRegionAt_InvalidAllowed(c)).Where(c => c != null);
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
				heuristicStrength = Mathf.RoundToInt(HeuristicStrengthHuman_DistanceCurve.Evaluate(lengthHorizontal));
			}
			closedCellCount = 0;
			openList.Clear();
			debug_pathFailMessaged = false;
			debug_totalOpenListCount = 0;
			debug_openCellsPopped = 0;
			moveTicksCardinal = pawn?.TicksPerMoveCardinal ?? 13;
			moveTicksDiagonal = pawn?.TicksPerMoveDiagonal ?? 18;

			RegionPathCostHeuristic regionCost = new RegionPathCostHeuristic(map, start, destinationRect, regions, traverseParms, moveTicksCardinal, moveTicksDiagonal);

			if (mode == HeuristicMode.Better)
			{
				if (canPassAnything)
				{
					//Roughly preserves the Vanilla behavior of increasing path accuracy for shorter paths and slower pawns, though not as smoothly. Only applies to sappers.
					heuristicStrength = Mathf.Max(1, Mathf.RoundToInt(heuristicStrength/(float) moveTicksCardinal));
				}
				else
				{	//Capped to 20,000 because otherwise long paths outside of allowed areas get ridiculous weightings
					var totalCostEst = Math.Min(regionCost.GetPathCostToRegion(curIndex), 20000);
					regionHeuristicWeight[1].x = totalCostEst / 2;
					regionHeuristicWeight[2].x = totalCostEst;
				}
			}
			calcGrid[curIndex].knownCost = 0;
			calcGrid[curIndex].heuristicCost = 0;
			calcGrid[curIndex].parentX = (ushort)start.x;
			calcGrid[curIndex].parentZ = (ushort)start.z;
			calcGrid[curIndex].status = statusOpenValue;
			openList.Push(new CostNode(curIndex, 0));
			Area area = null;
			if (pawn?.Drafted == false) {
				area = pawn?.playerSettings?.AreaRestrictionInPawnCurrentMap;
			}

			bool shouldCollideWithPawns = false;
			if (pawn != null) {
				shouldCollideWithPawns = PawnUtility.ShouldCollideWithPawns(pawn);
			}
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
					if (DebugViewSettings.drawPaths && !disableDebugFlash)
					{
						//draw backpointer
						var arrow = GetBackPointerArrow(calcGrid[curIndex].parentX, calcGrid[curIndex].parentZ, curX, curZ);
						string leading = "";
						string trailing = "";

#if DEBUG
						switch (calcGrid[curIndex].timesPopped)
						{
							case 1:
								trailing = "\n"; break;
							case 2: break;
							case 3:
								leading = "\n"; break;
							default:
								leading = "\n\n"; break;
						}
#endif
						DebugFlash(curIntVec3, calcGrid[curIndex].knownCost / 1500f, leading + calcGrid[curIndex].knownCost + " " + arrow + trailing);
					}
					if ((destinationIsOneCell && curIndex == destinationIndex) || destinationRect.Contains(curIntVec3))
					{
						PfProfilerEndSample();
						PfProfilerBeginSample("Finalize Path");
						var ret = FinalizedPath();
						PfProfilerEndSample();
						return ret;
					}
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
						neighX = (ushort) (curX + Directions[i]);
						neighZ = (ushort) (curZ + Directions[i + 8]);
						if (neighX >= mapSizeX || neighZ >= mapSizeZ) { continue; }
						neighIndex = cellIndices.CellToIndex(neighX, neighZ);
						if ((calcGrid[neighIndex].status != statusClosedValue) && (calcGrid[neighIndex].status != statusOpenValue))
						{
							#region edge cost
							calcGrid[neighIndex].edgeCost = 10000;
							int cost = 0;
							bool notWalkable = false;
							if (!pathGrid.WalkableFast(neighX, neighZ))
							{
								if (!canPassAnything)
								{
									//DebugFlash(intVec, 0.22f, "walk");
									continue;
								}
								notWalkable = true;
								cost += 60;
								Thing edifice = edificeGrid[neighIndex];
								if (edifice == null || !edifice.def.useHitPoints)
								{
									continue;
								}
								cost += (int) (edifice.HitPoints*0.1f);
							}
							if (i > 3)
							{
								switch (i)
								{
									case 4:
										if (!pathGrid.WalkableFast(curX, curZ - 1) || !pathGrid.WalkableFast(curX + 1, curZ))
										{
											continue;
										}
										break;
									case 5:
										if (!pathGrid.WalkableFast(curX, curZ + 1) || !pathGrid.WalkableFast(curX + 1, curZ))
										{
											continue;
										}
										break;
									case 6:
										if (!pathGrid.WalkableFast(curX, curZ + 1) || !pathGrid.WalkableFast(curX - 1, curZ))
										{
											continue;
										}
										break;
									case 7:
										if (!pathGrid.WalkableFast(curX, curZ - 1) || !pathGrid.WalkableFast(curX - 1, curZ))
										{
											continue;
										}
										break;
								}
							}
							neighCost = 0;
							neighCost += cost;
							if (!notWalkable)
							{
								neighCost += pathGridDirect[neighIndex];
							}
							if (avoidGrid != null)
							{
								neighCost += avoidGrid[neighIndex]*8;
							}
							if (area != null && !area[neighIndex])
							{
								neighCost += 600;
							}
							if (shouldCollideWithPawns && PawnUtility.AnyPawnBlockingPathAt(new IntVec3(neighX, 0, neighZ), pawn))
							{
								neighCost += 800;
							}
							Building building = edificeGrid[neighIndex];
							if (building != null)
							{
								PfProfilerBeginSample("Edifices");
								Building_Door building_Door = building as Building_Door;
								if (building_Door != null)
								{
									switch (traverseParms.mode)
									{
										case TraverseMode.ByPawn:
											if (!traverseParms.canBash && building_Door.IsForbiddenToPass(pawn))
											{
												if (DebugViewSettings.drawPaths)
												{
													DebugFlash(building.Position, 0.77f, "forbid");
												}
												PfProfilerEndSample();
												continue;
											}
											if (!building_Door.FreePassage)
											{
												if (building_Door.PawnCanOpen(pawn))
												{
													neighCost += building_Door.TicksToOpenNow;
												}
												else
												{
													if (!traverseParms.canBash)
													{
														if (DebugViewSettings.drawPaths)
														{
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
											if (!building_Door.FreePassage)
											{
												PfProfilerEndSample();
												continue;
											}
											break;
									}
								}
								else if (pawn != null)
								{
									neighCost += building.PathFindCostFor(pawn);
								}
								PfProfilerEndSample();
							}
							//Some mods can result in negative path costs. That'll work well enough with Vanilla, since it won't revisit closed nodes, but when we do, it's an infinite loop.
							calcGrid[neighIndex].edgeCost = (ushort)Mathf.Max(neighCost, 1);
							#endregion
#if DEBUG
							calcGrid[neighIndex].timesPopped = 0;
#endif
							#region heuristic
							PfProfilerBeginSample("Heuristic");
							switch (mode)
							{
								case HeuristicMode.Vanilla:
									h = heuristicStrength * (Mathf.Abs(neighX - destinationX) + Mathf.Abs(neighZ - destinationZ));
									break;
								case HeuristicMode.AdmissableOctile:
									{
										var dx = Mathf.Abs(neighX - destinationX);
										var dy = Mathf.Abs(neighZ - destinationZ);
										h = moveTicksCardinal * (dx + dy) + (moveTicksDiagonal - 2 * moveTicksCardinal) * Mathf.Min(dx, dy);
									}
									break;
								case HeuristicMode.Better:
									if (canPassAnything)
									{
										var dx = Mathf.Abs(neighX - destinationX);
										var dy = Mathf.Abs(neighZ - destinationZ);
										h = heuristicStrength * (moveTicksCardinal * (dx + dy) + (moveTicksDiagonal - 2 * moveTicksCardinal) * Mathf.Min(dx, dy));
									}
									else
									{
										h = regionCost.GetPathCostToRegion(neighIndex);
									}
									break;
							}
							calcGrid[neighIndex].heuristicCost = h;
							PfProfilerEndSample();
							#endregion
						}

						if (calcGrid[neighIndex].edgeCost < 10000)
						{
							neighIndexes[i] = neighIndex;
						}
					}
					#region BPMX Best H
					PfProfilerBeginSample("BPMX Best H");
					int bestH = calcGrid[curIndex].heuristicCost;
					if (mode == HeuristicMode.Better)
					{
						for (int i = 0; i < 8; i++)
						{
							neighIndex = neighIndexes[i];
							if (neighIndex < 0)
							{
								continue;
							}
							bestH = Math.Max(bestH, calcGrid[neighIndex].heuristicCost - (calcGrid[neighIndex].edgeCost + (i > 3 ? moveTicksDiagonal : moveTicksCardinal)));
						}
					}

					//Pathmax Rule 3
					calcGrid[curIndex].heuristicCost = bestH;
					PfProfilerEndSample();
					#endregion

					#region Pathmax rules
					for (int i = 0; i < 8; i++)
					{
						neighIndex = neighIndexes[i];
						if (neighIndex < 0) { continue; }
						if (calcGrid[neighIndex].status == statusClosedValue && (canPassAnything || mode != HeuristicMode.Better))
						{
							continue;
						}


						var thisDirEdgeCost = calcGrid[neighIndex].edgeCost + (i > 3 ? moveTicksDiagonal : moveTicksCardinal);
						neighCostThroughCur = thisDirEdgeCost + calcGrid[curIndex].knownCost;
						//Pathmax Rule 1
						int nodeH = mode == HeuristicMode.Better ? Mathf.Max(calcGrid[neighIndex].heuristicCost, bestH - thisDirEdgeCost) : calcGrid[neighIndex].heuristicCost;
						if (calcGrid[neighIndex].status == statusClosedValue || calcGrid[neighIndex].status == statusOpenValue)
						{
							bool needsUpdate = false;
							int minReopenGain = 0;
							if (calcGrid[neighIndex].status == statusOpenValue)
							{
								needsUpdate = nodeH > calcGrid[neighIndex].heuristicCost;
							}
							else
							{	//Don't reopen closed nodes if the path isn't cheaper by at least the difference between one straight & diagonal movement
								minReopenGain = moveTicksDiagonal - moveTicksCardinal;
							}
							calcGrid[neighIndex].heuristicCost = nodeH;
							
							if (!(neighCostThroughCur + minReopenGain < calcGrid[neighIndex].knownCost))
							{
								if (needsUpdate) //if the heuristic cost was increased for an open node, we need to adjust its spot in the queue
								{
									openList.PushOrUpdate(new CostNode(neighIndex, calcGrid[neighIndex].knownCost + Mathf.CeilToInt(nodeH * regionHeuristicWeight.Evaluate(nodeH))));
								}
								continue;
							}
						}

#if DEBUG
						if (calcGrid[neighIndex].status == statusClosedValue)
						{
							debug_closedCellsReopened++;
						}
#endif

						calcGrid[neighIndex].parentX = curX;
						calcGrid[neighIndex].parentZ = curZ;
						
						calcGrid[neighIndex].knownCost = neighCostThroughCur;
						calcGrid[neighIndex].status = statusOpenValue;
						calcGrid[neighIndex].heuristicCost = nodeH;
						
						PfProfilerBeginSample("Push Open");
						openList.PushOrUpdate(new CostNode(neighIndex, neighCostThroughCur + Mathf.CeilToInt(nodeH * regionHeuristicWeight.Evaluate(nodeH))));
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
				Log.Warning(string.Concat(pawn, " pathing from ", start, " to ", dest, " ran out of cells to process.\nJob:", text, "\nFaction: ", text2, "\n\nThis will be the last message to avoid spam."));
				debug_pathFailMessaged = true;
			}
			PfProfilerEndSample();
			return PawnPath.NotFound;
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
