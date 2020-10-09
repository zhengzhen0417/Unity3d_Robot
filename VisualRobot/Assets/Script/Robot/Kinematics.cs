using UnityEngine;
using System;
using System.Collections.Generic;
//by zz
namespace robot
{
    public class Kinematics
    {
        private const double mpi = 3.1415926;
        //DH参数
        static private double[,] DHParm = new double[,] {
        //offset    d         a           阿尔法
        { -mpi/2 ,  197.30 ,  0       ,   0},
        { -mpi/2 ,  200    ,  0       ,   -mpi/2},
        { 0      ,  -174.5 ,  809.2   ,   0},
        { mpi/2  ,  128.8  ,  720.2   ,   0},
        { 0      ,  113    ,  0       ,   mpi/2},
        { mpi/2  ,  104.5  ,  0       ,   -mpi/2}
        };

        static public double[] ToolPama = new double[] { 0, 0, 60, 0, 0, 0 };


        static double square(double m1)
        {
            return m1 * m1;
        }

        static double rad2deg(double rad)
        {
            return (180 / mpi) * rad;

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
            matrix.m02 = 0;
            matrix.m03 = (float)a;
            //第二行
            matrix.m10 = (float)(Math.Cos(alpha) * Math.Sin(theta));
            matrix.m11 = (float)(Math.Cos(alpha) * Math.Cos(theta));
            matrix.m12 = (float)-Math.Sin(alpha);
            matrix.m13 = (float)(-d * Math.Sin(alpha));
            //第三行
            matrix.m20 = (float)(Math.Sin(alpha) * Math.Sin(theta));
            matrix.m21 = (float)(Math.Sin(alpha) * Math.Cos(theta));
            matrix.m22 = (float)(Math.Cos(alpha));
            matrix.m23 = (float)(d * Math.Cos(alpha));
            //第四行
            matrix.m30 = 0;
            matrix.m31 = 0;
            matrix.m32 = 0;
            matrix.m33 = 1;

            return matrix;
        }

        //运动学正解
        static public void forwardKinematics(double q1, double q2, double q3, double q4, double q5, double q6, out Vector3 pos, out Vector3 pose)
        {
            //初始化旋转矩阵
            Matrix4x4 matrix01 = initT(deg2rad(q1) + DHParm[0, 0], DHParm[0, 1], DHParm[0, 2], DHParm[0, 3]);
            Matrix4x4 matrix12 = initT(deg2rad(q2) + DHParm[1, 0], DHParm[1, 1], DHParm[1, 2], DHParm[1, 3]);
            Matrix4x4 matrix23 = initT(deg2rad(q3) + DHParm[2, 0], DHParm[2, 1], DHParm[2, 2], DHParm[2, 3]);
            Matrix4x4 matrix34 = initT(deg2rad(q4) + DHParm[3, 0], DHParm[3, 1], DHParm[3, 2], DHParm[3, 3]);
            Matrix4x4 matrix45 = initT(deg2rad(q5) + DHParm[4, 0], DHParm[4, 1], DHParm[4, 2], DHParm[4, 3]);
            Matrix4x4 matrix56 = initT(deg2rad(q6) + DHParm[5, 0], DHParm[5, 1], DHParm[5, 2], DHParm[5, 3]);
            //初始化工具参数
            Quaternion quaternion = Quaternion.Euler(new Vector3((float)ToolPama[3], (float)ToolPama[4], (float)ToolPama[5]));
            Vector3 toolsPos = new Vector3((float)ToolPama[0], (float)ToolPama[1], (float)ToolPama[2]);
            Matrix4x4 mattool = Matrix4x4.TRS(toolsPos, quaternion, Vector3.one); ;
            //矩阵相乘
            Matrix4x4 matrix06 = matrix01 * matrix12 * matrix23 * matrix34 * matrix45 * matrix56 * mattool;
            //计算位置
            pos = new Vector3(matrix06.m03, matrix06.m13, matrix06.m23);
            //计算姿态
            var m = matrix06.rotation;
            pose = m.eulerAngles;
 
        }
        //运动学反解
        static public double[,] InverseKinematics(Vector3 Postion, Vector3 pose)
        {
            //DH参数
            double d1 = DHParm[0, 1];
            double d2 = DHParm[1, 1];
            double d3 = DHParm[2, 1];
            double d4 = DHParm[3, 1];
            double d5 = DHParm[4, 1];
            double d6 = DHParm[5, 1];
            double a3 = DHParm[2, 2];
            double a4 = DHParm[3, 2];
            //初始化旋转矩阵 TCP到法兰盘的转换
            Quaternion qTool = Quaternion.Euler(new Vector3((float)ToolPama[3], (float)ToolPama[4], (float)ToolPama[5]));
            Vector3 toolsPos = new Vector3((float)ToolPama[0], (float)ToolPama[1], (float)ToolPama[2]);
            Matrix4x4 mattool = Matrix4x4.TRS(toolsPos, qTool, Vector3.one);
            //初始化总的旋转矩阵 TCP到基座的转换
            Quaternion qbase2Tool = Quaternion.Euler(pose.x, pose.y, pose.z);
            Vector3 base2ToolPostion = Postion;

            Matrix4x4 Matbase2Tool = Matrix4x4.TRS(base2ToolPostion, qbase2Tool, Vector3.one);
            //计算基座到法兰盘中心的转换矩阵
            Matrix4x4 matBase2Plate = Matbase2Tool* Matrix4x4.Inverse(mattool);

            //PrintMat(matBase2Plate);
            //PrintMat(Matbase2Tool);
            //定义旋转矩阵的每行每一列 
            double nx = matBase2Plate.m00;
            double ny = matBase2Plate.m10;
            double nz = matBase2Plate.m20;

            double ox = matBase2Plate.m01;
            double oy = matBase2Plate.m11;
            double oz = matBase2Plate.m21;

            double ax = matBase2Plate.m02;
            double ay = matBase2Plate.m12;
            double az = matBase2Plate.m22;

            double px = matBase2Plate.m03;
            double py = matBase2Plate.m13;
            double pz = matBase2Plate.m23;

            //J1(2, 3)(2, 4)    
            double t11 = Math.Atan2(d2 + d3 + d4, Math.Sqrt(square(py - d6 * ay) + square(d6 * ax - px) - square(d2 + d3 + d4))) - Math.Atan2(py - d6 * ay, d6 * ax - px);
            double t12 = Math.Atan2(d2 + d3 + d4, -Math.Sqrt(square(py - d6 * ay) + square(d6 * ax - px) - square(d2 + d3 + d4))) - Math.Atan2(py - d6 * ay, d6 * ax - px);
            //% J5(2, 1)(2, 2)
            double t51 = Math.Atan2(Math.Sqrt(square(ny * Math.Cos(t11) - nx * Math.Sin(t11)) + square(oy * Math.Cos(t11) - ox * Math.Sin(t11))), ay * Math.Cos(t11) - ax * Math.Sin(t11));
            double t52 = Math.Atan2(-Math.Sqrt(square(ny * Math.Cos(t11) - nx * Math.Sin(t11)) + square(oy * Math.Cos(t11) - ox * Math.Sin(t11))), ay * Math.Cos(t11) - ax * Math.Sin(t11));
            double t53 = Math.Atan2(Math.Sqrt(square(ny * Math.Cos(t12) - nx * Math.Sin(t12)) + square(oy * Math.Cos(t12) - ox * Math.Sin(t12))), ay * Math.Cos(t12) - ax * Math.Sin(t12));
            double t54 = Math.Atan2(-Math.Sqrt(square(ny * Math.Cos(t12) - nx * Math.Sin(t12)) + square(oy * Math.Cos(t12) - ox * Math.Sin(t12))), ay * Math.Cos(t12) - ax * Math.Sin(t12));

            // % J6
            double t61 = Math.Atan2((ox * Math.Sin(t11) - oy * Math.Cos(t11)) / Math.Sin(t51), (ny * Math.Cos(t11) - nx * Math.Sin(t11)) / Math.Sin(t51));
            double t62 = Math.Atan2((ox * Math.Sin(t11) - oy * Math.Cos(t11)) / Math.Sin(t52), (ny * Math.Cos(t11) - nx * Math.Sin(t11)) / Math.Sin(t52));
            double t63 = Math.Atan2((ox * Math.Sin(t12) - oy * Math.Cos(t12)) / Math.Sin(t53), (ny * Math.Cos(t12) - nx * Math.Sin(t12)) / Math.Sin(t53));
            double t64 = Math.Atan2((ox * Math.Sin(t12) - oy * Math.Cos(t12)) / Math.Sin(t54), (ny * Math.Cos(t12) - nx * Math.Sin(t12)) / Math.Sin(t54));

            // % q234
            // % (1, 3)(3, 3)
            // % T16 = T12 * T23 * T34 * T45 * T56;
            //% TT = inv(T01) * mT;
            double t234_1 = Math.Atan2(az / Math.Sin(t51), -(ax * Math.Cos(t11) + ay * Math.Sin(t11)) / Math.Sin(t51));
            double t234_2 = Math.Atan2(az / Math.Sin(t52), -(ax * Math.Cos(t11) + ay * Math.Sin(t11)) / Math.Sin(t52));
            double t234_3 = Math.Atan2(az / Math.Sin(t53), -(ax * Math.Cos(t12) + ay * Math.Sin(t12)) / Math.Sin(t53));
            double t234_4 = Math.Atan2(az / Math.Sin(t54), -(ax * Math.Cos(t12) + ay * Math.Sin(t12)) / Math.Sin(t54));
            //% J2(1, 4)(3, 4)
            //% T15 = T12 * T23 * T34 * T45;
            //% TT = inv(T01) * mT * inv(T56);
            double A_1 = px * Math.Cos(t11) + py * Math.Sin(t11) - ay * d6 * Math.Sin(t11) - ax * d6 * Math.Cos(t11) - d5 * Math.Sin(t234_1);
            double A_2 = px * Math.Cos(t11) + py * Math.Sin(t11) - ay * d6 * Math.Sin(t11) - ax * d6 * Math.Cos(t11) - d5 * Math.Sin(t234_2);
            double A_3 = px * Math.Cos(t12) + py * Math.Sin(t12) - ay * d6 * Math.Sin(t12) - ax * d6 * Math.Cos(t12) - d5 * Math.Sin(t234_3);
            double A_4 = px * Math.Cos(t12) + py * Math.Sin(t12) - ay * d6 * Math.Sin(t12) - ax * d6 * Math.Cos(t12) - d5 * Math.Sin(t234_4);

            double B_1 = pz - d1 - az * d6 - d5 * Math.Cos(t234_1);
            double B_2 = pz - d1 - az * d6 - d5 * Math.Cos(t234_2);
            double B_3 = pz - d1 - az * d6 - d5 * Math.Cos(t234_3);
            double B_4 = pz - d1 - az * d6 - d5 * Math.Cos(t234_4);


            double t21 = Math.Atan2(square(a4) - square(a3) - square(A_1) - square(B_1), Math.Sqrt(4 * square(a3) * (square(A_1) + square(B_1)) - square(square(a4) - square(a3) - square(A_1) - square(B_1)))) + Math.Atan2(A_1, B_1);
            double t22 = Math.Atan2(square(a4) - square(a3) - square(A_1) - square(B_1), -Math.Sqrt(4 * square(a3) * (square(A_1) + square(B_1)) - square(square(a4) - square(a3) - square(A_1) - square(B_1)))) + Math.Atan2(A_1, B_1);
            double t23 = Math.Atan2(square(a4) - square(a3) - square(A_2) - square(B_2), Math.Sqrt(4 * square(a3) * (square(A_2) + square(B_2)) - square(square(a4) - square(a3) - square(A_2) - square(B_2)))) + Math.Atan2(A_2, B_2);
            double t24 = Math.Atan2(square(a4) - square(a3) - square(A_2) - square(B_2), -Math.Sqrt(4 * square(a3) * (square(A_2) + square(B_2)) - square(square(a4) - square(a3) - square(A_2) - square(B_2)))) + Math.Atan2(A_2, B_2);
            double t25 = Math.Atan2(square(a4) - square(a3) - square(A_3) - square(B_3), Math.Sqrt(4 * square(a3) * (square(A_3) + square(B_3)) - square(square(a4) - square(a3) - square(A_3) - square(B_3)))) + Math.Atan2(A_3, B_3);
            double t26 = Math.Atan2(square(a4) - square(a3) - square(A_3) - square(B_3), -Math.Sqrt(4 * square(a3) * (square(A_3) + square(B_3)) - square(square(a4) - square(a3) - square(A_3) - square(B_3)))) + Math.Atan2(A_3, B_3);
            double t27 = Math.Atan2(square(a4) - square(a3) - square(A_4) - square(B_4), Math.Sqrt(4 * square(a3) * (square(A_4) + square(B_4)) - square(square(a4) - square(a3) - square(A_4) - square(B_4)))) + Math.Atan2(A_4, B_4);
            double t28 = Math.Atan2(square(a4) - square(a3) - square(A_4) - square(B_4), -Math.Sqrt(4 * square(a3) * (square(A_4) + square(B_4)) - square(square(a4) - square(a3) - square(A_4) - square(B_4)))) + Math.Atan2(A_4, B_4);
            //% J23
            double t23_1 = Math.Atan2(-(B_1 + a3 * Math.Sin(t21)), A_1 - a3 * Math.Cos(t21));
            double t23_2 = Math.Atan2(-(B_1 + a3 * Math.Sin(t22)), A_1 - a3 * Math.Cos(t22));
            double t23_3 = Math.Atan2(-(B_2 + a3 * Math.Sin(t23)), A_2 - a3 * Math.Cos(t23));
            double t23_4 = Math.Atan2(-(B_2 + a3 * Math.Sin(t24)), A_2 - a3 * Math.Cos(t24));
            double t23_5 = Math.Atan2(-(B_3 + a3 * Math.Sin(t25)), A_3 - a3 * Math.Cos(t25));
            double t23_6 = Math.Atan2(-(B_3 + a3 * Math.Sin(t26)), A_3 - a3 * Math.Cos(t26));
            double t23_7 = Math.Atan2(-(B_4 + a3 * Math.Sin(t27)), A_4 - a3 * Math.Cos(t27));
            double t23_8 = Math.Atan2(-(B_4 + a3 * Math.Sin(t28)), A_4 - a3 * Math.Cos(t28));
            //% J3
            double t31 = t23_1 - t21;
            double t32 = t23_2 - t22;
            double t33 = t23_3 - t23;
            double t34 = t23_4 - t24;
            double t35 = t23_5 - t25;
            double t36 = t23_6 - t26;
            double t37 = t23_7 - t27;
            double t38 = t23_8 - t28;
            //% J4
            double t41 = t234_1 - t23_1;
            double t42 = t234_1 - t23_2;
            double t43 = t234_2 - t23_3;
            double t44 = t234_2 - t23_4;
            double t45 = t234_3 - t23_5;
            double t46 = t234_3 - t23_6;
            double t47 = t234_4 - t23_7;
            double t48 = t234_4 - t23_8;

            //计算出的8组解
            double[,] ikine_t = new double[,] {
            { t11, t21, t31, t41, t51 ,t61, },
            { t11, t22, t32, t42, t51, t61,},
            { t11, t23, t33, t43, t52, t62,},
            { t11, t24, t34, t44, t52, t62,},
            { t12, t25, t35, t45, t53, t63,},
            { t12, t26, t36, t46, t53, t63,},
            { t12, t27, t37, t47, t54, t64,},
            { t12, t28, t38, t48, t54, t64 }, };
            for (int i = 0; i < 8; i++)
            {
                for (int j = 0; j < 6; j++)
                {

                    var temp = ikine_t[i, j] - DHParm[j, 0];

                    if (temp < -mpi)
                    {
                        temp += 2*mpi;
                    }
                    else if (temp > mpi)
                    {

                        temp -= 2*mpi;
                    }
                    ikine_t[i, j] =rad2deg(temp);
                }
                
            }
            return ikine_t;
        }

        static void PrintIkine(double [,] ikine)
        {
            for (int i = 0; i < 8; i++)
            {
                String str= "Ans" + i.ToString();
                
                for (int j = 0; j < 6; j++)
                {
                    str += " " + ikine[i, j].ToString("0.00");
                   
                }
                
                Debug.Log(str);
            }
        }

       public static void PrintPose(Vector3 v1,Vector3 v2)
        {
            string str = v1.x.ToString("0.00") + "  " + v1.y.ToString("0.00") + "  " + v1.z.ToString("0.00") + "  " +
                v2.x.ToString("0.00") + "  " + v2.y.ToString("0.00") + "  " + v2.z.ToString("0.00") + "  ";
            Debug.Log(str);
        }

        static void PrintMat(Matrix4x4 mat,string name=null)
        {
            Debug.Log(name);
            for (int i = 0; i < 4; i++)
            {
                string str=null;

                for (int j = 0; j < 4; j++)
                {
                    str += " " + mat[i, j].ToString("0.00");

                }
                Debug.Log(str);
            }

        }
    }
}