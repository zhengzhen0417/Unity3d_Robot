using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace robot
{
    public class Axis
    {
        //转轴
        public UnityEngine.Vector3 AxisCenter;
        //偏移
        public float Offset;
        //Tran
        public Transform Trans;
        //dir 
        public bool Dir;

        
        public Axis(Transform transform, UnityEngine.Vector3 AxisCenter,float offset,bool dir)
        {
            this.AxisCenter = AxisCenter;
            this.Offset = offset;
            this.Trans = transform;
            this.Dir = dir;

        }
        public void SetAngle(float angle)
        {
            Trans.localEulerAngles=new UnityEngine.Vector3(Trans.localEulerAngles.x, Trans.localEulerAngles.y, Dir? angle : -angle - Offset);                       
        }

        public float GetAngle()
        {
            float angle = Dir ? (Trans.localEulerAngles.z - Offset): (-Trans.localEulerAngles.z - Offset);
            if (angle < -180)
                angle += 360;



            return angle;
        }
    }
}
