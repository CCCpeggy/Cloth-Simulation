using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace Partical
{
    public class ClothSpring : MonoBehaviour
    {
        public ClothParticle p1, p2;      // the indices of the particles the spring connects
        public Matrix<double> Jx = Utility.CreateMatrix3x3();    // Jacobian with respect to position
        public Matrix<double> Jv = Utility.CreateMatrix3x3();    // Jacobian with resepct to velocity

        public double r;        // rest length
        public double ks = 1;       // stiffness constant (aka. spring constant)
        public double kd = 0.1;       // damping constant
                                // Start is called before the first frame update
        private LineRenderer line;
        void Start()
        {
            line = gameObject.AddComponent<LineRenderer>();
        }

        // Update is called once per frame
        void Update()
        {
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