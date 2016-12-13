using System;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Text;

namespace BetterPathfinding
{
	class Utils
	{
		public static Func<TObject, TValue> GetFieldAccessor<TObject, TValue>(string fieldName)
		{
			ParameterExpression param = Expression.Parameter(typeof(TObject), "arg");
			MemberExpression member = Expression.Field(param, fieldName);
			LambdaExpression lambda = Expression.Lambda(typeof(Func<TObject, TValue>), member, param);
			Func<TObject, TValue> compiled = (Func<TObject, TValue>)lambda.Compile();
			return compiled;
		}
	}
}
