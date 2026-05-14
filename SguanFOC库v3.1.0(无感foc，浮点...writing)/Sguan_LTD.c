/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-05-14 22:17:01
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-05-14 22:18:29
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_LTD.c
 * @Description: SguanFOC库的“LTD最速控制函数”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_LTD.h"

/**
 * @description: 最速控制LTD函数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LTD_STRUCT} *ltd
 * @return {*}
 */
void LTD_Init(LTD_STRUCT *ltd){

}

/**
 * @description: 最速控制LTD的运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Hall.png
 * @reminder: (上方链接是此Sguan_LTD模块Simulink原理仿真图)
 * @param {LTD_STRUCT} *ltd
 * @return {*}
 */
void LTD_Loop(LTD_STRUCT *ltd){

}

