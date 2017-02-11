using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Xml;
using UnityEngine;
using Verse;

namespace BetterPathfinding
{

#if DEBUG
	// Mono performs float math at double precision. To get exactly the same results for a path while running in the MS .NET CLR,
	// a double SimpleCurve is necessary.
	internal class SimpleCurve : IEnumerable<CurvePoint>
	{
		private List<CurvePoint> points = new List<CurvePoint>();

		public CurvePoint this[int i] => points[i];

		IEnumerator IEnumerable.GetEnumerator()
		{
			return GetEnumerator();
		}

		[DebuggerHidden]
		public IEnumerator<CurvePoint> GetEnumerator()
		{
			foreach (CurvePoint point in points)
			{
				yield return point;
			}
		}

		public void Add(CurvePoint newPoint)
		{
			points.Add(newPoint);
			SortPoints();
		}

		public void SortPoints()
		{
			Comparison<CurvePoint> comparison = delegate (CurvePoint a, CurvePoint b)
			{
				if (a.x < b.x)
				{
					return -1;
				}
				return b.x < a.x ? 1 : 0;
			};
			points.Sort(comparison);
		}

		public double Evaluate(float x)
		{
			if (points.Count == 0)
			{
				Log.Error("Evaluating a SimpleCurve with no points.");
				return 0f;
			}
			if (x <= points[0].x)
			{
				return points[0].y;
			}
			if (x >= points[points.Count - 1].x)
			{
				return points[points.Count - 1].y;
			}
			CurvePoint curvePoint = points[0];
			CurvePoint curvePoint2 = points[points.Count - 1];
			for (int i = 0; i < points.Count; i++)
			{
				if (x <= points[i].x)
				{
					curvePoint2 = points[i];
					if (i > 0)
					{
						curvePoint = points[i - 1];
					}
					break;
				}
			}
			double t = (x - curvePoint.x) / (curvePoint2.x - curvePoint.x);
			return curvePoint.y + (curvePoint2.y - curvePoint.y) * t;
		}
	}

	internal class CurvePoint
	{
		public double x { get; set; }
		public double y { get; set; }

		public CurvePoint(float x, float y)
		{
			this.x = x;
			this.y = y;
		}
	}
#endif
}
