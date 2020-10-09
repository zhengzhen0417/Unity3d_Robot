using robot;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class TestFunction : MonoBehaviour
{

    public  AxisManage axisManage;

    //测试起始坐标
    Vector3 Position = new Vector3(173.9274f, 1152.5190f, 200.2073f);
    Vector3 pose = new Vector3(357.3308f, 180.4817f, 312.9065f);
    //PtP移动测试
    public void PtPMoveTest()
    {

        axisManage.Run = true;
        //添加划线函数
        Thread thread = new Thread(delegate () {
            axisManage.CMove(new CPostion(Position, pose), false);
            axisManage.CMove(new CPostion(Position + new Vector3(-300, 0, 0), pose), true);
            axisManage.CMove(new CPostion(Position + new Vector3(-300, 300, 0), pose), true);
            axisManage.CMove(new CPostion(Position + new Vector3(-300, 300, 300), pose), true);
            axisManage.CMove(new CPostion(Position + new Vector3(0, 0, 0), pose), true);
        });
        thread.Start();
    }
    //直线移动测试
    public void LineTest()
    {
        Position.x = 173.9274f;
        Position.y = 1152.5190f;
        Position.z = 200.2073f;

        pose.x = 357.3308f;
        pose.y = 180.4817f;
        pose.z = 312.9065f;


        axisManage.Run = true;
        //添加划线函数
        Thread thread = new Thread(delegate () {
            axisManage.CMove(new CPostion(Position, pose), false);
            axisManage.CLine(new CPostion(Position + new Vector3(-300, 0, 0), pose), true);
            axisManage.CLine(new CPostion(Position + new Vector3(-300, 300, 0), pose), true);
            axisManage.CLine(new CPostion(Position + new Vector3(-300, 300, 300), pose), true);
            axisManage.CLine(new CPostion(Position + new Vector3(0, 0, 0), pose), true);
        });
        thread.Start();
    }
    //写字测试
    public void WriteWordsTest()
    {
        //设置当前位置
        axisManage.Run = true;
        //添加划线函数
        Thread thread = new Thread(delegate () {
            axisManage.CMove(new CPostion(Position, pose), false);
            for (int i = 0; i < Data.DataBuffer.Length / 4; i++)
            {

                //PtPMove
                if (Data.DataBuffer[i, 3] == 0)
                {
                    axisManage.CMove(new CPostion(Position + new Vector3((float)Data.DataBuffer[i, 0], (float)Data.DataBuffer[i, 1], (float)Data.DataBuffer[i, 2] + 30), pose), false);
                }
                else//LineMove
                {

                    axisManage.CLine(new CPostion(Position + new Vector3((float)Data.DataBuffer[i, 0], (float)Data.DataBuffer[i, 1], (float)Data.DataBuffer[i, 2] + 30), pose), true);

                }
            }

            axisManage.CMove(new CPostion(Position + new Vector3(0, -200, 100), pose), false);
        });
        thread.Start();

    }

   
}
