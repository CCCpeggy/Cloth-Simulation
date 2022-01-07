using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace Partical
{
    public class ClothParticle : MonoBehaviour
    {
        public Vector<double> x = Utility.CreateVector3d();  // position
        public Vector<double> v = Utility.CreateVector3d();  // velocity
        public Vector<double> F = Utility.CreateVector3d();  // force
        public double m = 1;   // the mass
                           // Start is called before the first frame update
        public int index;
        public bool isPin = false;
        void Start()
        {
        }

        // Update is called once per frame
        void Update()
        {
            transform.position = new Vector3((float)x[0], (float)x[1], (float)x[2]);
        }
    }
}