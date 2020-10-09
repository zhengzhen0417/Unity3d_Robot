using robot;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PostionDisplay : MonoBehaviour
{
    // Start is called before the first frame update
    private const float m2mm = 1000;

    //直接获取的 世界坐标

    public List<Text> txt_Gpos;

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

    //按钮控制
    public List<Button> Ctrl_ButtonAdd;
    public List<Button> Ctrl_ButtonSub;
    //构型选择 下拉框 姿态选择
    public Dropdown dropdown_PoseChoose;


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
    private void UpdateGPos()
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

        Kinematics.forwardKinematics(q1, q2, q3, q4, q5, q6, out Pos, out pose);
        txt_Cpos[0].text = "X:" + (Pos.x).ToString("0.0000") + "mm";
        txt_Cpos[1].text = "Y:" + (Pos.y).ToString("0.0000") + "mm";
        txt_Cpos[2].text = "Z:" + (Pos.z).ToString("0.0000") + "mm";
        txt_Cpos[3].text = "RX:" + (pose.x).ToString("0.0000") + "°";
        txt_Cpos[4].text = "RY:" + (pose.y).ToString("0.0000") + "°";
        txt_Cpos[5].text = "RZ:" + (pose.z).ToString("0.0000") + "°";

    }

    //跟新机器人关节坐标系
    private void UpdateJPos()
    {
        int i = 0;
        foreach (var item in txt_Jpos)
        {
            item.text = "J" + (i + 1).ToString() + ":" + axisManage.AxisList[i].GetAngle().ToString("0.00");
            i++;
        }
    }
    //构型切换事件
    private void ChangePose(int value)
    {
        Vector3 Postion ;
        Vector3 Pose ;
        //运动学正解计算出姿态
        Kinematics.forwardKinematics(axisManage.Axis1.GetAngle(),
                                     axisManage.Axis2.GetAngle(),
                                     axisManage.Axis3.GetAngle(),
                                     axisManage.Axis4.GetAngle(),
                                     axisManage.Axis5.GetAngle(),
                                     axisManage.Axis6.GetAngle(),
                                     out Postion,
                                     out Pose
            );

        //运动学反解计算出8组解
        var kk = Kinematics.InverseKinematics(Postion, Pose);
        //判断角度是否有效
        if (kk[value, 0] == kk[value, 0] &&
            kk[value, 1] == kk[value, 1] &&
            kk[value, 2] == kk[value, 2] &&
            kk[value, 3] == kk[value, 3] &&
            kk[value, 4] == kk[value, 4] &&
            kk[value, 5] == kk[value, 5]
            )
        {
            axisManage.SetJPos((float)kk[value, 0], (float)kk[value, 1], (float)kk[value, 2], (float)kk[value, 3], (float)kk[value, 4], (float)kk[value, 5]);
        }
        else
        {
            Debug.LogError("当前构型不存在");
        }      
    }

    private void Start()
    {

        //添加滑块事件
        slider_Axis[0].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis1));
        slider_Axis[1].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis2));
        slider_Axis[2].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis3));
        slider_Axis[3].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis4));
        slider_Axis[4].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis5));
        slider_Axis[5].onValueChanged.AddListener((float value) => OnSliderValueChange(value, axisManage.Axis6));
        //按钮添加事件
        Ctrl_ButtonAdd[0].onClick.AddListener(delegate (){axisManage.XMove(10);});
        Ctrl_ButtonAdd[1].onClick.AddListener(delegate () { axisManage.YMove(10); });
        Ctrl_ButtonAdd[2].onClick.AddListener(delegate () { axisManage.ZMove(10); });

        Ctrl_ButtonSub[0].onClick.AddListener(delegate () { axisManage.XMove(-10); });
        Ctrl_ButtonSub[1].onClick.AddListener(delegate () { axisManage.YMove(-10); });
        Ctrl_ButtonSub[2].onClick.AddListener(delegate () { axisManage.ZMove(-10); });

        //添加构型切换世间
        dropdown_PoseChoose.onValueChanged.AddListener((int value)=> ChangePose(value));

        //设置起始位置
        slider_Axis[0].value = -1.05f;
        slider_Axis[1].value = -37.79f;
        slider_Axis[2].value = -93.74f;
        slider_Axis[3].value = 44.19f;
        slider_Axis[4].value = 90.53f;
        slider_Axis[5].value = 41.88f;
    }

    private void OnSliderValueChange(float value, Axis axis)
    {
        axis.SetAngle(value);
    }

    // Update is called once per frame
    private void Update()
    {
        //1 更新机器人世界坐标
        UpdateGPos();
        //2 更新机器人关节坐标系
        UpdateJPos();
        //更新世界坐标系
        UpdateCPos();
    }
}