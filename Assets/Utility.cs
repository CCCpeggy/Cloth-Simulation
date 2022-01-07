using UnityEngine;
using System.Collections.Generic;
using UnityEngine.Assertions;
using MathNet.Numerics.LinearAlgebra;

namespace Partical
{
    public class Utility
    {
        public static Vector<double> CreateVector3d(double n1=0, double n2=0, double n3=0)
        {
            return Vector<double>.Build.Dense(new[] { n1, n2, n3 });
        }
        public static Matrix<double> CreateMatrix3x3(
            double n11=0, double n12=0, double n13=0,
            double n21=0, double n22=0, double n23=0,
            double n31=0, double n32=0, double n33=0)
        {
            return Matrix<double>.Build.Dense(3, 3, new[] { n11, n12, n13, n21, n22, n23, n31, n32, n33 });
        }
        public static Matrix<double> CreateMatrixd3nx3n(int n, bool isDiagonal = false)
        {
            if (isDiagonal)
                return Matrix<double>.Build.DenseDiagonal(3 * n, 3 * n);
            return Matrix<double>.Build.Dense(3 * n, 3 * n);
        }
        public static Vector<double> CreateVectord3n(int n)
        {
            return Vector<double>.Build.Dense(3 * n);
        }
        public static Matrix<double> OuterProduct(Vector<double> v, Vector<double> u)
        {

            return Matrix<double>.Build.Dense(3, 3, new[] {
                v[0] * u[0], v[0] * u[1], v[0] * u[2],
                v[1] * u[0], v[1] * u[1], v[1] * u[2],
                v[2] * u[0], v[2] * u[1], v[2] * u[2]}
            );
        }
        public static double InnerProduct(Vector<double> v, Vector<double> u)
        {
            return v[0] * u[0] + v[1] * u[1] + v[2] * u[2];
        }
        public static Matrix<double> GetIdentity()
        {
            return Matrix<double>.Build.DenseIdentity(3, 3);
        }
        public static void PutMatrix3IntoMatrix(Matrix<double> m1, int i, int j, Matrix<double> m2)
        {
            for (int ii = 0; ii < 3; ii++)
            {
                for (int jj = 0; jj < 3; jj++)
                {
                    m1[i * 3 + ii, j * 3 + jj] = m2[ii, jj];
                    m1[i * 3 + ii, j * 3 + jj] = m2[ii, jj];
                    m1[i * 3 + ii, j * 3 + jj] = m2[ii, jj];
                }
            }
        }
        public static void PutVector3IntoMatrix(Matrix<double> matrix, int i, int j, Vector<double> vector)
        {
            matrix[i * 3 + 0, j * 3 + 0] = vector[0];
            matrix[i * 3 + 1, j * 3 + 1] = vector[1];
            matrix[i * 3 + 2, j * 3 + 2] = vector[2];
        }
        public static void PutVector3IntoVector(Vector<double> v1, int i, Vector<double> v2)
        {
            v1[i * 3 + 0] = v2[0];
            v1[i * 3 + 1] = v2[1];
            v1[i * 3 + 2] = v2[2];
        }
        public static Vector<double> GetVector3FromVector(Vector<double> v, int i)
        {
            return CreateVector3d(v[i * 3 + 0], v[i * 3 + 1], v[i * 3 + 2]);
        }
        // public static void CopyValue(Vector<double> f, Vector<double> d)
        // {
        //     if (f.Count != d.Count) {
        //         Debug.LogError("目標 vector 與來源 vector 的長度不同");
        //         return;
        //     }
        //     for (int i = 0; i < f.Count; i++) {
        //         d[i] = f[i];
        //     }
        // }
    }
}
