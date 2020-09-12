using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class measure : MonoBehaviour
{
    //机器人基座
    public GameObject rbt_base;
    //机器人工具
    public GameObject rbt_tool;
    // Start is called before the first frame update
    void Start()
    {
        var  rbtbasePos= rbt_base.transform.position;
        var rbttoolPos = rbt_tool.transform.position;
        var Dpos = rbtbasePos - rbttoolPos;
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
