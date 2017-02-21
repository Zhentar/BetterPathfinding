using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Linq.Expressions;
using System.Text;
using Verse;

namespace BetterPathfinding
{
	public static class Utils
	{
		public static Func<TObject, TValue> GetFieldAccessor<TObject, TValue>(string fieldName)
		{
			ParameterExpression param = Expression.Parameter(typeof(TObject), "arg");
			MemberExpression member = Expression.Field(param, fieldName);
			LambdaExpression lambda = Expression.Lambda(typeof(Func<TObject, TValue>), member, param);
			Func<TObject, TValue> compiled = (Func<TObject, TValue>)lambda.Compile();
			return compiled;
		}

		public static Func<object, TValue> GetFieldAccessor<TValue>(Type objectType, string fieldName)
		{
			var param = Expression.Parameter(typeof(object), "arg");
			var cast = Expression.Convert(param, objectType);
			var member = Expression.Field(cast, fieldName);
			var lambda = Expression.Lambda(typeof(Func<object, TValue>), member, param);
			var compiled = (Func<object, TValue>)lambda.Compile();
			return compiled;
		}

		public static bool IsDiagonal(this IntVec3 a, IntVec3 b)
		{
			return a.x != b.x && a.z != b.z;
		}

		public static bool WalkableExtraFast(this int[] pathGridDirect, int index)
		{
			return pathGridDirect[index] < 10000;
		}

		public static bool IsIndexDiagonal(this int a, int b, Map map)
		{
			return ((a  % map.Size.x != b % map.Size.x)) && ((a / map.Size.x != b / map.Size.x));
		}

		public static IEnumerable<int> NeighborIndices(this int index, Map map)
		{
			int mapX = map.Size.x;
			var eastInBounds = (index % mapX > 0);
			var westInBounds = (index % mapX < (mapX - 1));
			if (index > mapX) //North in bounds
			{
				yield return index - mapX;
				if (eastInBounds) { yield return index - mapX - 1; }
				if (westInBounds) { yield return index - mapX + 1; }
			}
			if (eastInBounds) { yield return index - 1; }
			if (westInBounds) { yield return index + 1; }
			if ((index / mapX) < (map.Size.z - 1)) //South in bounds
			{
				yield return index + mapX;
				if (eastInBounds) { yield return index + mapX - 1; }
				if (westInBounds) { yield return index + mapX + 1; }
			}
		}

		public static IEnumerable<int> PathableNeighborIndices(this int index, Map map)
		{
			int mapX = map.Size.x;
			var eastInBounds = (index % mapX > 0) && map.pathGrid.pathGrid.WalkableExtraFast(index - 1);
			var westInBounds = (index % mapX < (mapX - 1)) && map.pathGrid.pathGrid.WalkableExtraFast(index + 1);
			if (index > mapX && map.pathGrid.pathGrid.WalkableExtraFast(index - mapX)) //North in bounds
			{
				yield return index - mapX;
				if (eastInBounds) { yield return index - mapX - 1; }
				if (westInBounds) { yield return index - mapX + 1; }
			}
			if (eastInBounds) { yield return index - 1; }
			if (westInBounds) { yield return index + 1; }
			if ((index / mapX) < (map.Size.z - 1) && map.pathGrid.pathGrid.WalkableExtraFast(index + mapX)) //South in bounds
			{
				yield return index + mapX;
				if (eastInBounds) { yield return index + mapX - 1; }
				if (westInBounds) { yield return index + mapX + 1; }
			}
		}

#if DEBUG
		[StaticConstructorOnStartup]
		public static class Log
		{
			static Log()
			{
				OpenStream();
			}

			public class LogStream
			{
				public string fileName;
				public FileStream stream;
				public int indent;
			}

			public const string cclLogFileName = "pathfind_log.txt";
			private static LogStream cclStream;

			public static LogStream OpenStream(string filename = cclLogFileName)
			{
				var newStream = new LogStream();
				newStream.fileName = filename;
				newStream.indent = 0;
				newStream.stream = System.IO.File.Open(filename, System.IO.FileMode.Create, System.IO.FileAccess.Write,
					System.IO.FileShare.Read);
				if (filename == cclLogFileName)
				{
					cclStream = newStream;
				}
				return newStream;
			}

			public static void CloseStream(LogStream stream = null)
			{
				if (stream == null)
				{
					stream = cclStream;
				}
				if (stream != null)
				{
					stream.stream.Close();
					stream.stream = null;
					stream = null;
				}
			}

			public static void IndentStream(LogStream stream = null, int amount = 1)
			{
				if (stream == null)
				{
					stream = cclStream;
				}
				if (stream != null)
				{
					stream.indent += amount;
					if (stream.indent < 0)
					{
						stream.indent = 0;
					}
				}
			}

			public static void Write(string s, LogStream stream = null)
			{
				if (
					(s.NullOrEmpty()) ||
					(
						(stream == null) &&
						(cclStream == null)
					)
				)
				{
					return;
				}

				if (stream == null)
				{
					stream = cclStream;
				}

				s += "\n";
				// Copy to a byte array with preceeding tabs for indentation
				byte[] b = new byte[stream.indent + s.Length];

				if (stream.indent > 0)
				{
					for (int i = 0; i < stream.indent; ++i)
					{
						b[i] = 9; // Tab
					}
				}

				for (int i = 0; i < s.Length; ++i)
				{
					b[stream.indent + i] = (byte) s[i];
				}

				stream.stream.Write(b, 0, stream.indent + s.Length);
				stream.stream.Flush();
			}
		}
		#endif
	}
}
