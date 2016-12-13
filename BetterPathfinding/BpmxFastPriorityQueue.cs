using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BetterPathfinding
{

	//This class is based on the Vanilla FastPriorityQueue, with the addition of reprioritization
	class BpmxFastPriortyQueue
	{
		private readonly int[] gridIndexToQueueIndex;

		public BpmxFastPriortyQueue(IComparer<NewPathFinder.CostNode> comparer, int gridSize)
		{
			gridIndexToQueueIndex = new int[gridSize];
			this.comparer = comparer;
		}

		public void PushOrUpdate(NewPathFinder.CostNode item)
		{
			var queueIndex = gridIndexToQueueIndex[item.gridIndex];
			if (queueIndex < innerList.Count && innerList[queueIndex].gridIndex == item.gridIndex)
			{
				var oldCost = innerList[queueIndex].totalCostEstimate;
				innerList[queueIndex] = item;
				if (oldCost < item.totalCostEstimate)
				{
					CascadeDown(queueIndex);
				}
				else
				{
					CascadeUp(queueIndex);
				}
			}
			else
			{
				Push(item);
			}
		}

		private List<NewPathFinder.CostNode> innerList = new List<NewPathFinder.CostNode>();

		private IComparer<NewPathFinder.CostNode> comparer;

		public int Count => innerList.Count;

		public void Push(NewPathFinder.CostNode item)
		{
			int num = innerList.Count;
			innerList.Add(item);
			gridIndexToQueueIndex[item.gridIndex] = num;
			CascadeUp(num);
		}

		private void CascadeUp(int index)
		{
			while (index != 0)
			{
				int num2 = (index - 1) / 2;
				if (CompareElements(index, num2) >= 0)
				{
					return;
				}
				SwapElements(index, num2);
				index = num2;
			}
		}

		public NewPathFinder.CostNode Pop()
		{
			var result = innerList[0];
			innerList[0] = innerList[innerList.Count - 1];
			gridIndexToQueueIndex[innerList[0].gridIndex] = 0;
			innerList.RemoveAt(innerList.Count - 1);
			CascadeDown(0);
			return result;
		}

		private void CascadeDown(int index)
		{
			while (true)
			{
				int num2 = index;
				int num3 = 2 * index + 1;
				int num4 = 2 * index + 2;
				if (innerList.Count > num3 && CompareElements(index, num3) > 0)
				{
					index = num3;
				}
				if (innerList.Count > num4 && CompareElements(index, num4) > 0)
				{
					index = num4;
				}
				if (index == num2)
				{
					break;
				}
				SwapElements(index, num2);
			}
		}

		public void Clear()
		{
			innerList.Clear();
		}

		private void SwapElements(int i, int j)
		{
			var value = innerList[i];
			innerList[i] = innerList[j];
			innerList[j] = value;
			gridIndexToQueueIndex[innerList[i].gridIndex] = i;
			gridIndexToQueueIndex[innerList[j].gridIndex] = j;
		}

		private int CompareElements(int i, int j)
		{
			return comparer.Compare(innerList[i], innerList[j]);
		}
	}
}
