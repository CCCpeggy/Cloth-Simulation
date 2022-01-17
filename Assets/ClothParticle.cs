using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using UnityEditor;

namespace Partical
{
    // 這是衣服上的粒子
    public class ClothParticle : MonoBehaviour
    {
        // Implicit 使用的是 double 計算，所以這邊都使用 Numberics Library 來儲存
        public Vector<double> x = Utility.CreateVector3d();  // 位置
        public Vector<double> v = Utility.CreateVector3d();  // 速度
        public Vector<double> F = Utility.CreateVector3d();  // 力
        public double m = 1;   // 重量 (但有些解法我沒有拿這個 m)
        public int index;   // 粒子編號
        bool isPin = false; // 位置是否鎖定
        // 狀態是否鎖定 (除了本身設定外，還有選取的時候也會鎖定)
        bool statusPin = false;
        // 對外的鎖定設置與取得
        public bool IsPin {
            set{
                isPin=value;
                statusPin=value;
            }
            get{
                return statusPin;
            }
        }
        void Start()
        {
        }

        public void Update()
        {
            // 判斷是否被選取
            if (Selection.Contains(gameObject)) {
                // 選取狀態時，將的位置更新置，並且作為鎖定的粒子
                x[0] = transform.position.x; x[1] = transform.position.y; x[2] = transform.position.z;
                statusPin = true;
            }
            else {
                // 將算出來的結果更新至粒子實際位置
                transform.position = new Vector3((float)x[0], (float)x[1], (float)x[2]);
                statusPin = isPin;
            }
        }

    }
}