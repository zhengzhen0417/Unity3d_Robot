using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
namespace robot
{
    //世界坐标
   public class CPostion
    {
        public float x;
        public float y;
        public float z;
        public float rx;
        public float ry;
        public float rz;

        public Vector3 Postion {
            get { return new Vector3((float)x,(float)y,(float)z); }
            
        }
        public Vector3 Pose
        {
            get { return new Vector3((float)rx, (float)ry, (float)rz); }

        }

        public CPostion(float x, float y, float z, float rx, float ry , float rz)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.rx = rx;
            this.ry = ry;
            this.rz = rz;
        }
        public CPostion(Vector3 Pos,Vector3 pose)
        {
            this.x = Pos.x;
            this.y = Pos.y;
            this.z = Pos.z;
            this.rx = pose.x;
            this.ry = pose.y;
            this.rz = pose.z;
        }

        public double GetDistance(CPostion pose)
        {
           return Math.Sqrt(
                (pose.x-x)*(pose.x - x)+
                (pose.y - y)*(pose.y - y)+
                (pose.z - z)*(pose.z - z)+
                (pose.rx - rx)*(pose.rx - rx)+
                (pose.ry - ry)*(pose.ry - ry)+
                (pose.rz - rz)*(pose.rz - rz)
                );
        }



    }
}
