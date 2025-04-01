using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.Collections;

/// <summary>
/// 优先队列实现
/// </summary>

namespace PathPlanning
{
    /// <summary>
    /// 优先队列实现
    /// </summary>
    public class PriorityQueue<T>
    {
        private List<(T Item, float Priority)> elements = new List<(T, float)>();
        private Dictionary<T, int> indexMap = new Dictionary<T, int>();
        
        public int Count => elements.Count;
        
        public void Enqueue(T item, float priority)
        {
            elements.Add((item, priority));
            indexMap[item] = elements.Count - 1;
            BubbleUp(elements.Count - 1);
        }
        
        public T Dequeue()
        {
            if (elements.Count == 0)
                throw new InvalidOperationException("Queue is empty");
                
            T item = elements[0].Item;
            
            // 将最后一个元素移到顶部
            int lastIndex = elements.Count - 1;
            elements[0] = elements[lastIndex];
            elements.RemoveAt(lastIndex);
            
            // 更新索引映射
            indexMap.Remove(item);
            
            if (elements.Count > 0)
            {
                indexMap[elements[0].Item] = 0;
                BubbleDown(0);
            }
            
            return item;
        }
        
        public bool TryDequeue(out T item, out float priority)
        {
            if (elements.Count == 0)
            {
                item = default;
                priority = default;
                return false;
            }
            
            item = elements[0].Item;
            priority = elements[0].Priority;
            
            // 将最后一个元素移到顶部
            int lastIndex = elements.Count - 1;
            elements[0] = elements[lastIndex];
            elements.RemoveAt(lastIndex);
            
            // 更新索引映射
            indexMap.Remove(item);
            
            if (elements.Count > 0)
            {
                indexMap[elements[0].Item] = 0;
                BubbleDown(0);
            }
            
            return true;
        }
        
        public bool Contains(T item)
        {
            return indexMap.ContainsKey(item);
        }
        
        public void UpdatePriority(T item, float newPriority)
        {
            if (!indexMap.TryGetValue(item, out int index))
                throw new ArgumentException("Item not found in queue");
                
            float oldPriority = elements[index].Priority;
            elements[index] = (item, newPriority);
            
            if (newPriority < oldPriority)
                BubbleUp(index);
            else if (newPriority > oldPriority)
                BubbleDown(index);
        }
        
        private void BubbleUp(int index)
        {
            while (index > 0)
            {
                int parentIndex = (index - 1) / 2;
                
                if (elements[index].Priority >= elements[parentIndex].Priority)
                    break;
                    
                Swap(index, parentIndex);
                index = parentIndex;
            }
        }
        
        private void BubbleDown(int index)
        {
            int lastIndex = elements.Count - 1;
            
            while (true)
            {
                int leftChildIndex = 2 * index + 1;
                int rightChildIndex = 2 * index + 2;
                int smallestIndex = index;
                
                // 找出三个节点中优先级最小的
                if (leftChildIndex <= lastIndex && elements[leftChildIndex].Priority < elements[smallestIndex].Priority)
                    smallestIndex = leftChildIndex;
                    
                if (rightChildIndex <= lastIndex && elements[rightChildIndex].Priority < elements[smallestIndex].Priority)
                    smallestIndex = rightChildIndex;
                    
                if (smallestIndex == index)
                    break;
                    
                Swap(index, smallestIndex);
                index = smallestIndex;
            }
        }
        
        private void Swap(int i, int j)
        {
            var temp = elements[i];
            elements[i] = elements[j];
            elements[j] = temp;
            
            // 更新索引映射
            indexMap[elements[i].Item] = i;
            indexMap[elements[j].Item] = j;
        }
    }
}