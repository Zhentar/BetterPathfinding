using System;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Text;
using UnityEngine;
using Verse;

namespace BetterPathfinding
{
	internal class DebugCell
	{
		public static Func<object, TValue> GetFieldAccessor<TValue>(Type objectType, string fieldName)
		{
			var param = Expression.Parameter(typeof(object), "arg");
			var cast = Expression.Convert(param, objectType);
			var member = Expression.Field(cast, fieldName);
			var lambda = Expression.Lambda(typeof(Func<object, TValue>), member, param);
			var compiled = (Func<object, TValue>)lambda.Compile();
			return compiled;
		}

		static DebugCell()
		{
			var debugCellType = typeof(GameInitData).Assembly.GetType("Verse.DebugCell");
			cGetter = GetFieldAccessor<IntVec3>(debugCellType, "c");
			displayStringGetter = GetFieldAccessor<string>(debugCellType, "displayString");
		}

		private static readonly Func<object, IntVec3> cGetter;

		private static readonly Func<object, string> displayStringGetter;

		public void OnGUI()
		{
			var displayString = displayStringGetter(this);
			if (displayString != null)
			{
				Vector2 vector = cGetter(this).ToUIPosition();
				if (vector.x > 0 && vector.y > 0 && vector.x < Screen.width && vector.y < Screen.height)
				{
					Rect rect = new Rect(vector.x - 35, vector.y - 35, 70f, 70f);
					Widgets.Label(rect, displayString);
				}
			}
		}
	}
}
