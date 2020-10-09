using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using robot;
using System.Reflection;
using System.Threading;
using UnityEngine.UI;

public class AxisManage : MonoBehaviour
{
    // Start is called before the first frame update
    //机器人基座
    public GameObject rbt_base;
    //机器人工具
    public GameObject rbt_tool;
    //材质
    public Material Ballmaterial;
    //小球
    public GameObject Ball;
    //轴
    public GameObject rbt_Axis1;
    public GameObject rbt_Axis2;
    public GameObject rbt_Axis3;
    public GameObject rbt_Axis4;
    public GameObject rbt_Axis5;
    public GameObject rbt_Axis6;
    //速度
    public Slider slider_Speed;
    //
    public Axis Axis1;
    public Axis Axis2;
    public Axis Axis3;
    public Axis Axis4;
    public Axis Axis5;
    public Axis Axis6;

    public List<Axis> AxisList;
    //轴的偏差
    const float offset1=0;
    const float offset2 =0;
    const float offset3 =0;
    const float offset4 =0;
    const float offset5 =0;
    const float offset6 =0;

    //目标位置
    Vector3 mAxis1_3target = new Vector3();
    Vector3 mAxis4_6target = new Vector3();
    //运动标志
    public bool Run=false;
    //命令缓冲区
    Queue PosList = new Queue();
    //虚拟的坐标
    Vector3 Axis1_3Visual = new Vector3();
    Vector3 Axis4_6Visual = new Vector3();
    //插补长度 5mm 5mm插补一次
    const float interLine = 5;
    //速度
    public float speed = 1;
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
        slider_Speed.onValueChanged.AddListener((float value) => { speed = value;});
    }
    //直接设置各个轴角度
    public void SetJPos(double q1, double q2, double q3, double q4, double q5, double q6)
    {
        Axis1.SetAngle((float)q1);
        Axis2.SetAngle((float)q2);
        Axis3.SetAngle((float)q3);
        Axis4.SetAngle((float)q4);
        Axis5.SetAngle((float)q5);
        Axis6.SetAngle((float)q6);
    }
    //沿世界坐标系移动到目标位置 PtP移动
    public void CMove(CPostion pose,bool CreateBall = false)
    {
        Vector3 Axis1_3target=new Vector3();
        Vector3 Axis4_6target = new Vector3();
        //运动学正解计算出8组解
        var kk = Kinematics.InverseKinematics(pose.Postion, pose.Pose);

        for (int i = 0; i < 8; i++)
        {
            //判断角度是否有效
            if (kk[i, 0] == kk[i, 0] &&
                kk[i, 1] == kk[i, 1] &&
                kk[i, 2] == kk[i, 2] &&
                kk[i, 3] == kk[i, 3] &&
                kk[i, 4] == kk[i, 4] &&
                kk[i, 5] == kk[i, 5]
                )
            {               
                Axis1_3target = new Vector3((float)kk[i, 0], (float)kk[i, 1], (float)kk[i, 2]);
                Axis4_6target = new Vector3((float)kk[i, 3], (float)kk[i, 4], (float)kk[i, 5]);
                JPostion jp = new JPostion(Axis1_3target, Axis4_6target);
                jp.CreateBall = CreateBall;
                lock (PosList)
                {
                    PosList.Enqueue(jp);                
                }
                break;
            }
        }
        Axis1_3Visual.x = Axis1_3target.x;
        Axis1_3Visual.y = Axis1_3target.y;
        Axis1_3Visual.z = Axis1_3target.z;
        Axis4_6Visual.x = Axis4_6target.x;
        Axis4_6Visual.y = Axis4_6target.y;
        Axis4_6Visual.z = Axis4_6target.z;
    }
    //沿世界坐标系移动到目标位置 直线移动
    public void CLine(CPostion mpose,bool CreateBall=false)
    {
        Vector3 Position;
        Vector3 pose;
        Vector3 Axis1_3target = new Vector3();
        Vector3 Axis4_6target = new Vector3();
        var kk = Kinematics.InverseKinematics(mpose.Postion, mpose.Pose);
        //运动学正解计算出当前位置，
        Kinematics.forwardKinematics(Axis1_3Visual.x, Axis1_3Visual.y, Axis1_3Visual.z, Axis4_6Visual.x, Axis4_6Visual.y, Axis4_6Visual.z,out Position,out pose);
        //计算两者之间直线长度
        double distance= mpose.GetDistance(new CPostion(Position, pose));
        //计算插值数量
        int count = (int)(distance / interLine);
        //计算单个轴的增量
        //如果距离很小
        if (count == 0)
        {
            var ikine_t = Kinematics.InverseKinematics(mpose.Postion, mpose.Pose);
            for (int i = 0; i < 8; i++)
            {
                //判断角度是否有效
                if (ikine_t[i, 0] == ikine_t[i, 0] &&
                    ikine_t[i, 1] == ikine_t[i, 1] &&
                    ikine_t[i, 2] == ikine_t[i, 2] &&
                    ikine_t[i, 3] == ikine_t[i, 3] &&
                    ikine_t[i, 4] == ikine_t[i, 4] &&
                    ikine_t[i, 5] == ikine_t[i, 5]
                    )
                {


                    Axis1_3target = new Vector3((float)ikine_t[i, 0], (float)ikine_t[i, 1], (float)ikine_t[i, 2]);
                    Axis4_6target = new Vector3((float)ikine_t[i, 3], (float)ikine_t[i, 4], (float)ikine_t[i, 5]);

                    JPostion jp = new JPostion(Axis1_3target, Axis4_6target);
                    jp.CreateBall = CreateBall;
                    lock (PosList)
                    {
                        PosList.Enqueue(jp);

                    }
                    break;
                }
            }
        }
        else
        {
            double dx = (mpose.x - Position.x) / count;
            double dy = (mpose.y - Position.y) / count;
            double dz = (mpose.z - Position.z) / count;
            double drx = (mpose.rx - pose.x) / count;
            double dry = (mpose.ry - pose.y) / count;
            double drz = (mpose.rz - pose.z) / count;
            //细分
            for (int j = 0; j < count; j++)
            {
                Position.x += (float)dx;

                Position.y += (float)dy;
                Position.z += (float)dz;
                pose.x += (float)drx;
                pose.y += (float)dry;
                pose.z += (float)drz;
                //计算逆解
                var ikine_t = Kinematics.InverseKinematics(Position, pose);

                for (int i = 0; i < 8; i++)
                {
                    //判断角度是否有效
                    if (ikine_t[i, 0] == ikine_t[i, 0] &&
                        ikine_t[i, 1] == ikine_t[i, 1] &&
                        ikine_t[i, 2] == ikine_t[i, 2] &&
                        ikine_t[i, 3] == ikine_t[i, 3] &&
                        ikine_t[i, 4] == ikine_t[i, 4] &&
                        ikine_t[i, 5] == ikine_t[i, 5]
                        )
                    {


                        Axis1_3target = new Vector3((float)ikine_t[i, 0], (float)ikine_t[i, 1], (float)ikine_t[i, 2]);
                        Axis4_6target = new Vector3((float)ikine_t[i, 3], (float)ikine_t[i, 4], (float)ikine_t[i, 5]);
                        JPostion jp = new JPostion(Axis1_3target, Axis4_6target);
                        jp.CreateBall = CreateBall;
                        lock (PosList)
                        {
                            PosList.Enqueue(jp);

                        }
                        break;
                    }
                }

            }

        }
       


        //更新当前虚拟坐标
        Axis1_3Visual.x = Axis1_3target.x;
        Axis1_3Visual.y = Axis1_3target.y;
        Axis1_3Visual.z = Axis1_3target.z;
        Axis4_6Visual.x = Axis4_6target.x;
        Axis4_6Visual.y = Axis4_6target.y;
        Axis4_6Visual.z = Axis4_6target.z;
    }
    private void XYZMove(float dx,float dy,float dz)
    {
        Vector3 Position;
        Vector3 pose;

        double q1 = Axis1.GetAngle();
        double q2 = Axis2.GetAngle();
        double q3 = Axis3.GetAngle();
        double q4 = Axis4.GetAngle();
        double q5 = Axis5.GetAngle();
        double q6 = Axis6.GetAngle();

        Kinematics.forwardKinematics(q1, q2, q3, q4, q5, q6, out Position, out pose);
        Position.x = Position.x + dx;
        Position.y = Position.y + dy;
        Position.z = Position.z + dz;
        var kk = Kinematics.InverseKinematics(Position, pose);

        for (int i = 0; i < 6; i++)
        {
            if (kk[i, 0] == kk[i, 0] &&
                kk[i, 1] == kk[i, 1] &&
                kk[i, 2] == kk[i, 2] &&
                kk[i, 3] == kk[i, 3] &&
                kk[i, 4] == kk[i, 4] &&
                kk[i, 5] == kk[i, 5]
                )
            {
                SetJPos(kk[i, 0], kk[i, 1], kk[i, 2], kk[i, 3], kk[i, 4], kk[i, 5]);
                GameObject obj1 = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                obj1.GetComponent<MeshRenderer>().material = Ballmaterial;
                //设置物体的位置Vector3三个参数分别代表x,y,z的坐标数  
                obj1.transform.position = rbt_tool.transform.position;
                obj1.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
                obj1.transform.parent = Ball.transform;
                break;
            }
        }
    }
    public void XMove(float dx)
    {
        XYZMove(dx,0,0);
    }
    public void YMove(float dy)
    {
        XYZMove(0, dy, 0);
    }
    public void ZMove(float dz)
    {
        XYZMove(0, 0, dz);
    }  
    bool Finsh = true;//当前小段是否走完
    bool CreateBall = false;//是否创建小球
    private void FixedUpdate()
    {
        if (Run)
        {
            if (Finsh == true)
            {
                Finsh = false;
                //从队列中获取一个元素
                JPostion jp = (JPostion)PosList.Dequeue();
                CreateBall = jp.CreateBall;
                mAxis1_3target = jp.Axis1_3;
                mAxis4_6target = jp.Axis4_6;
            }

            Vector3 Axis1_3 = new Vector3(Axis1.GetAngle(), Axis2.GetAngle(), Axis3.GetAngle());
            Vector3 Axis4_6 = new Vector3(Axis4.GetAngle(), Axis5.GetAngle(), Axis6.GetAngle());
            var temp1_3 = Vector3.MoveTowards(Axis1_3, mAxis1_3target, speed);
            var temp4_6 = Vector3.MoveTowards(Axis4_6, mAxis4_6target, speed);
            SetJPos(temp1_3.x, temp1_3.y, temp1_3.z, temp4_6.x, temp4_6.y, temp4_6.z); 
            //创建一个小球
            if (CreateBall)
            {
                GameObject obj1 = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                obj1.GetComponent<MeshRenderer>().material = Ballmaterial;
                //设置物体的位置Vector3三个参数分别代表x,y,z的坐标数  
                obj1.transform.position = rbt_tool.transform.position;
                obj1.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
                obj1.transform.parent = Ball.transform;

            }
            //判断当前小步是否走完
            if ((Axis1_3 - mAxis1_3target).magnitude < 0.001f
                && (Axis4_6 - mAxis4_6target).magnitude < 0.001f
                )
            {
                Finsh = true;

                if (PosList.Count == 0)
                {
                    Run = false;
                    Debug.Log("运动完成");
                } 
            }
        }
    }
    //清除小球
    public void ClearBall()
    {
        foreach (Transform child in Ball.transform)
        {
            Destroy(child.gameObject);
        }
    }
    
  
}
