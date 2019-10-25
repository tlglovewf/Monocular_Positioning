//
//  M_Config.h
//  
//
//  Created by TuLigen on 2019/6/4.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef _M_CONFIG_H_H_
#define _M_CONFIG_H_H_

#include "M_Types.h"
/*
 * 配置接口类
 */
class  IConfig
{
public:
	IConfig(){}
	virtual ~IConfig(){}


	/*
	 * 读取配置
	 */
	virtual bool ReadConfig(Camera &cam) = 0;
};

/*
 * 威亚标定参数读取
 */
class  WeiYaConfig : public IConfig
{
public:
	/* 构造函数
	 * @param intrin 内参文件
	 * @param bs     安置参数
	 */
	WeiYaConfig(const std::string &intrin, const std::string &bs) :mIntrinsics(intrin), mBs(bs)
	{
	}
    ~WeiYaConfig()
    {}
	/*
	* 读取配置
	*/
	virtual bool ReadConfig(Camera &cam);

protected:
	std::string mIntrinsics;
	std::string mBs;
};


//add more

#endif
