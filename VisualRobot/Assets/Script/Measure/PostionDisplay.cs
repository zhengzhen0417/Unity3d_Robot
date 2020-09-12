using robot;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PostionDisplay : MonoBehaviour
{
    // Start is called before the first frame update
    const float m2mm = 1000;

    //直接获取的 世界坐标


    public  List<Text> txt_Gpos;
    //关节坐标
    public List<Text> txt_Jpos;
    //设置角度坐标
    public List<Slider> slider_Axis;
    //计算出的笛卡尔坐标系 正解
    public List<Text> txt_Cpos;

    //
    public AxisManage axisManage;

    



    //机器人基座
    public GameObject rbt_base;
    //机器人工具
    public GameObject rbt_tool;

    private Vector3 GetRelativePosition(Transform origin, Vector3 position)
    {
        Vector3 distance = position - origin.position;
        Vector3 relativePosition = Vector3.zero;
        relativePosition.x = Vector3.Dot(distance, origin.right.normalized);
        relativePosition.y = Vector3.Dot(distance, origin.up.normalized);
        relativePosition.z = Vector3.Dot(distance, origin.forward.normalized);
        return relativePosition;
    }
    //更新机器人的坐标
    void UpdateGPos()
    {
        //Unity 坐标系不遵循右手定则

        var rbtbasePos = rbt_base.transform.position;
        var rbttoolPos = rbt_tool.transform.position;
        var Dpos = GetRelativePosition(rbt_base.transform, rbttoolPos);
        //var angle = rbt_base.transform.rotation- rbt_tool.transform.rotation;
       // Quaternion
        txt_Gpos[0].text = "X:" + (Dpos.y * m2mm).ToString("0.00") + "mm";
        txt_Gpos[1].text = "Y:" + (Dpos.x * m2mm).ToString("0.00") + "mm";
        txt_Gpos[2].text = "Z:" + (Dpos.z * m2mm).ToString("0.00") + "mm";

        


    }
    private void UpdateCPos()
    {
        Vector3 Pos;
        Vector3 pose;
        double q1 = axisManage.Axis1.GetAngle();
        double q2 = axisManage.Axis2.GetAngle();
        double q3 = axisManage.Axis3.GetAngle();
        double q4 = axisManage.Axis4.GetAngle();
        double q5 = axisManage.Axis5.GetAngle();
        double q6 = axisManage.Axis6.GetAngle();

        Kinematics.forwardKinematics(q1,q2,q3 ,q4,q5,q6,out Pos,out pose);
        txt_Cpos[0].text = "X:" + (Pos.x).ToString("0.00") + "mm";
        txt_Cpos[1].text = "Y:" + (Pos.y).ToString("0.00") + "mm";
        txt_Cpos[2].text = "Z:" + (Pos.z).ToString("0.00") + "mm";
        txt_Cpos[3].text = "RX:" + (pose.x).ToString("0.00") + "°";
        txt_Cpos[4].text = "RY:" + (pose.y).ToString("0.00") + "°";
        txt_Cpos[5].text = "RZ:" + (pose.z).ToString("0.00") + "°";




    }
    //跟新机器人关节坐标系
    void UpdateJPos()
    {
        int i = 0;
        foreach (var item in txt_Jpos)
        {
            item.text ="J"+(i+1).ToString()+ ":"+axisManage.AxisList[i].GetAngle().ToString("0.00");
            i++;
            
        }
    }
    private void Awake()
    {
        //添加滑块事件
        slider_Axis[0].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis1));
        slider_Axis[1].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis2));
        slider_Axis[2].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis3));
        slider_Axis[3].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis4));
        slider_Axis[4].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis5));
        slider_Axis[5].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis6));
    }



    private void OnSliderValueChange(float value,Axis axis)
    {
        axis.SetAngle(value);
        //Debug.Log(value);
    }





    // Update is called once per frame
    void Update()
    {
        //1 更新机器人世界坐标
        UpdateGPos();
        //2 更新机器人关节坐标系
        UpdateJPos();
        //更新世界坐标系
        UpdateCPos();
    }
}
