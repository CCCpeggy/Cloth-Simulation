using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace Partical
{
    // 這是一個彈簧，會連接兩個點，並儲存對應的參數
    // 彈簧特性可以參考這個影片：https://www.youtube.com/watch?v=kyQP4t_wOGI&t=531s&ab_channel=Gonkee
    public class ClothSpring : MonoBehaviour
    {
        public ClothParticle p1, p2;      // the indices of the particles the spring connects
        public Matrix<double> Jx = Utility.CreateMatrix3x3();    // Jacobian with respect to position (Implicit 用)
        public Matrix<double> Jv = Utility.CreateMatrix3x3();    // Jacobian with resepct to velocity (Implicit 用)
        public double r;            // rest length
        public double ks = 10;      // stiffness constant (aka. spring constant)
        public double kd = 1;       // damping constant
        private LineRenderer line;  // 將粒子之間化線
        void Start()
        {
            line = gameObject.AddComponent<LineRenderer>();
        }

        // Update is called once per frame
        void Update()
        {
            // 更新位置
            line.SetPosition(0, p1.transform.position);
            line.SetPosition(1, p2.transform.position);
        }

        public void Setup(ClothParticle p1, ClothParticle p2, double r) {
            this.p1 = p1;
            this.p2 = p2;
            this.r = r;
        }
    }
}