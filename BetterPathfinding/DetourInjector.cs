using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using Verse;
using Verse.AI;

namespace BetterPathfinding
{
	[StaticConstructorOnStartup]
	internal static class DetourInjector
	{
		private static Assembly Assembly => Assembly.GetAssembly(typeof(DetourInjector));

		private static string AssemblyName => Assembly.FullName.Split(',').First();

		static DetourInjector()
		{
			LongEventHandler.QueueLongEvent(Inject, "Initializing", true, null);

			#if DEBUG

			if (Prefs.DevMode)
			{
				DebugViewSettings.drawPaths = true;
			}

			#endif
		}

		private static void Inject()
		{
			if (DoInject())
				Log.Message(AssemblyName + " injected.");
			else
				Log.Error(AssemblyName + " failed to get injected properly.");
		}

		private const BindingFlags UniversalBindingFlags = BindingFlags.Static | BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

		private static bool DoInject()
		{
			MethodInfo RimWorld_PathFinder_FindPath = typeof(Verse.AI.PathFinder).GetMethod("FindPath", new [] { typeof(IntVec3), typeof(LocalTargetInfo), typeof(TraverseParms), typeof(PathEndMode) });
			MethodInfo ModTest_PathFinder_FindPath = typeof(PathFinderDetour).GetMethod("FindPath", UniversalBindingFlags);
			if (!Detours.TryDetourFromTo(RimWorld_PathFinder_FindPath, ModTest_PathFinder_FindPath))
				return false;

#if DEBUG
            var assembly = typeof(GameInitData).Assembly;
			var debugCellType = assembly.GetType("Verse.DebugCell");
			if (!DoDetour(debugCellType, typeof(DebugCell), "OnGUI")) return false;
#endif
            return true;
		}


		private static bool DoDetour(Type rimworld, Type mod, string method)
		{
			MethodInfo RimWorld_A = rimworld.GetMethod(method, UniversalBindingFlags);
			MethodInfo ModTest_A = mod.GetMethod(method, UniversalBindingFlags);
			if (!Detours.TryDetourFromTo(RimWorld_A, ModTest_A))
				return false;
			return true;
		}

	}

	#region Injection Code & MapComponent
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

}
