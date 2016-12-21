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


			var assembly = typeof(GameInitData).Assembly;
			var debugCellType = assembly.GetType("Verse.DebugCell");
			if (!DoDetour(debugCellType, typeof(DebugCell), "OnGUI")) return false;

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
}
