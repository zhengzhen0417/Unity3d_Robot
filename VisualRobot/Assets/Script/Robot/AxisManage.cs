using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using robot;
using System.Reflection;

public class AxisManage : MonoBehaviour
{
    // Start is called before the first frame update
    //机器人基座
    public GameObject rbt_base;
    //机器人工具
    public GameObject rbt_tool;
    //轴
    public GameObject rbt_Axis1;
    public GameObject rbt_Axis2;
    public GameObject rbt_Axis3;
    public GameObject rbt_Axis4;
    public GameObject rbt_Axis5;
    public GameObject rbt_Axis6;


    //
    public Axis Axis1;
    public Axis Axis2;
    public Axis Axis3;
    public Axis Axis4;
    public Axis Axis5;
    public Axis Axis6;

    public List<Axis> AxisList;
    //
    const float offset1=0;
    const float offset2 =0;
    const float offset3 =0;
    const float offset4 =0;
    const float offset5 =0;
    const float offset6 =0;

    //获取到旋转的正确数值

    private void Awake()
    {
        Vector3 nn = rbt_Axis2.transform.localEulerAngles;

        Axis1 = new Axis(rbt_Axis1.transform, Vector3.forward, offset1,false);
        Axis2 = new Axis(rbt_Axis2.transform, Vector3.forward, offset2, false);
        Axis3 = new Axis(rbt_Axis3.transform, Vector3.forward, offset3, false);
        Axis4 = new Axis(rbt_Axis4.transform, Vector3.forward, offset4, false);
        Axis5 = new Axis(rbt_Axis5.transform, Vector3.forward, offset5, false);
        Axis6 = new Axis(rbt_Axis6.transform, Vector3.forward, offset6, false);
        AxisList = new List<Axis> { Axis1, Axis2, Axis3, Axis4, Axis5, Axis6 };
    }




    // Update is called once per frame
    void Update()
    {



        //Kinematics.forwardKinematics(q1,q2,q3,q4,q5,q6);


    }


    public void setangle()
    {
        Vector3 pos = new Vector3(3.8f,1156,179);
        Vector3 pose = new Vector3(179,0,132);
         Kinematics.InverseKinematics(pos, pose);
    }
}
