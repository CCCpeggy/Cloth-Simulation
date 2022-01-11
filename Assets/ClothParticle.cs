using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using UnityEditor;

namespace Partical
{
    public class ClothParticle : MonoBehaviour
    {
        public Vector<double> x = Utility.CreateVector3d();  // position
        public Vector<double> v = Utility.CreateVector3d();  // velocity
        public Vector<double> F = Utility.CreateVector3d();  // force
        public double m = 1;   // the mass
        public int index;
        bool isPin = false;
        public bool IsPin {
            set{
                isPin=value;
            }
            get{
                return statusPin;
            }
        }
        bool statusPin = false;
        void Start()
        {
        }


        // Update is called once per frame
        void Update()
        {
            if (Selection.Contains(gameObject)) {
                x[0] = transform.position.x; x[1] = transform.position.y; x[2] = transform.position.z;
                statusPin = true;
            }
            else {
                transform.position = new Vector3((float)x[0], (float)x[1], (float)x[2]);
                statusPin = isPin;
            }
        }
    }
}