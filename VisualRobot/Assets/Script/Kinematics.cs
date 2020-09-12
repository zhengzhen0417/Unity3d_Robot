using UnityEngine;
using System;
using System.Collections.Generic;
//by zz
namespace robot
{
    public class Kinematics
    {
        private const double mpi = 3.1415926;
        //正向运动学

       static private double[,] DHParm = new double[,] {
        //offset    d         a           阿尔法
        { -mpi/2 ,  197.30 ,  0       ,   0},
        { -mpi/2 ,  200    ,  0       ,   -mpi/2},
        { 0      ,  -174.5 ,  809.2   ,   0},
        { mpi/2  ,  128.8  ,  720.2   ,   0},
        { 0      ,  113    ,  0       ,   mpi/2},
        { mpi/2  ,  104.5  ,  0       ,   -mpi/2}
        };

        static public double[] ToolPama = new double[] {0,0,60,0,0,0};
            

       static double rad2deg(double rad)
        {
            return 180 * mpi * rad;

        }
       static double deg2rad(double deg)
        {
            return deg * mpi / 180;
        }
       static Matrix4x4 initT(double theta, double d, double a, double alpha)
        {
            Matrix4x4 matrix = new Matrix4x4();
            //第一行
            matrix.m00 = (float)Math.Cos(theta);
            matrix.m01 = (float)-Math.Sin(theta);
            matrix.m02 =0;
            matrix.m03 = (float)a;
            //第二行
            matrix.m10 = (float)(Math.Cos(alpha) * Math.Sin(theta));
            matrix.m11 = (float)(Math.Cos(alpha) * Math.Cos(theta));
            matrix.m12 = (float)-Math.Sin(alpha);
            matrix.m13 = (float)(-d * Math.Sin(alpha));
            //第三行
            matrix.m20 = (float)(Math.Sin(alpha) * Math.Sin(theta));
            matrix.m21 = (float)(Math.Sin(alpha)*Math.Cos(theta));
            matrix.m22 = (float)(Math.Cos(alpha));
            matrix.m23 = (float)(d*Math.Cos(alpha));
            //第四行
            matrix.m30 = 0;
            matrix.m31 = 0;
            matrix.m32 = 0;
            matrix.m33 = 1;

            return matrix;
        }
        //运动学正解
       static public void forwardKinematics(double q1, double q2, double q3, double q4, double q5, double q6,out Vector3 pos,out Vector3 pose)
        {
            //初始化旋转矩阵
            Matrix4x4 matrix01 = initT(deg2rad(q1)+ DHParm[0,0], DHParm[0, 1], DHParm[0, 2], DHParm[0, 3]);
            Matrix4x4 matrix12 = initT(deg2rad(q2) + DHParm[1, 0], DHParm[1, 1], DHParm[1, 2], DHParm[1, 3]);
            Matrix4x4 matrix23 = initT(deg2rad(q3) + DHParm[2, 0], DHParm[2, 1], DHParm[2, 2], DHParm[2, 3]);
            Matrix4x4 matrix34 = initT(deg2rad(q4) + DHParm[3, 0], DHParm[3, 1], DHParm[3, 2], DHParm[3, 3]);
            Matrix4x4 matrix45 = initT(deg2rad(q5) + DHParm[4, 0], DHParm[4, 1], DHParm[4, 2], DHParm[4, 3]);
            Matrix4x4 matrix56 = initT(deg2rad(q6) + DHParm[5, 0], DHParm[5, 1], DHParm[5, 2], DHParm[5, 3]);           
            //初始化工具参数
            Quaternion quaternion=Quaternion.Euler(new Vector3((float)ToolPama[3], (float)ToolPama[4], (float)ToolPama[5]));           
            Vector3 toolsPos = new Vector3((float)ToolPama[0], (float)ToolPama[1], (float)ToolPama[2]);
            Matrix4x4 mattool = Matrix4x4.TRS(toolsPos, quaternion, Vector3.one); ;
            //矩阵相乘
            Matrix4x4 matrix06 = matrix01 * matrix12 * matrix23 * matrix34 * matrix45 * matrix56* mattool;
            //计算位置
            pos = new Vector3(matrix06.m03,matrix06.m13,matrix06.m23);
            //计算姿态
            var m = matrix06.rotation;
            pose = m.eulerAngles;
        }
        //运动学反解
        static public void InverseKinematics(Vector3 Postion,Vector3 pose)
        {
            //DH参数
            double d1 = DHParm[0,1];
            double d2 = DHParm[1, 1];
            double d3 = DHParm[2, 1];
            double d4 = DHParm[3, 1];
            double d5 = DHParm[4, 1];
            double d6 = DHParm[5, 1];
            double a3 = DHParm[2, 2];
            double a4 = DHParm[3, 3];
            //初始化旋转矩阵
            Quaternion quaternion = Quaternion.Euler(new Vector3((float)ToolPama[3], (float)ToolPama[4], (float)ToolPama[5]));
            Vector3 toolsPos = new Vector3((float)ToolPama[0], (float)ToolPama[1], (float)ToolPama[2]);
            Matrix4x4 mattool = Matrix4x4.TRS(toolsPos, quaternion, Vector3.one);



        }
    }
}