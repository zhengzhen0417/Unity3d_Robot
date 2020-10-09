using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace robot
{
    //角度坐标
   public class JPostion
    {
        public float q1;
        public float q2;
        public float q3;
        public float q4;
        public float q5;
        public float q6;
        public bool CreateBall=false;

        //Action 
        public Vector3 Axis1_3 {
            get { return new Vector3(q1, q2, q3); }
        }
        public Vector3 Axis4_6
        {
            get { return new Vector3(q4, q5, q6); }
        }


        public JPostion(float q1,float q2,float q3,float q4,float q5 ,float q6)
        {
            this.q1 = q1;
            this.q2 = q2;
            this.q3 = q3;
            this.q4 = q4;
            this.q5 = q5;
            this.q6 = q6;
        }
        public JPostion(Vector3 v1, Vector3 v2)
        {
            this.q1 = v1.x;
            this.q2 = v1.y;
            this.q3 = v1.z;
            this.q4 = v2.x;
            this.q5 = v2.y;
            this.q6 = v2.z;
        }




    }
}
