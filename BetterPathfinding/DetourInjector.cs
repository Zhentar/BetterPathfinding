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
			MethodInfo RimWorld_PathFinder_FindPath = typeof(Verse.AI.PathFinder).GetMethod("FindPath", new [] { typeof(IntVec3), typeof(TargetInfo), typeof(TraverseParms), typeof(PathEndMode) });
			MethodInfo ModTest_PathFinder_FindPath = typeof(PathFinder).GetMethod("_FindPath", UniversalBindingFlags);
			if (!Detours.TryDetourFromTo(RimWorld_PathFinder_FindPath, ModTest_PathFinder_FindPath))
				return false;


			MethodInfo RimWorld_PathFinder_Reinit = typeof(Verse.AI.PathFinder).GetMethod("Reinit", UniversalBindingFlags);
			MethodInfo ModTest_PathFinder_Reinit = typeof(PathFinder).GetMethod("_Reinit", UniversalBindingFlags);
			if (!Detours.TryDetourFromTo(RimWorld_PathFinder_Reinit, ModTest_PathFinder_Reinit))
				return false;

			return true;
		}

	}
}
