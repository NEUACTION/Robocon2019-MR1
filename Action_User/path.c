#include "path.h"
#include "robot.h"

#define MAX_PLAN_VEL PULSE_2_VEL(MAX_MOTOR_SPEED)

void PathLineInterpolation(Pose_t *path , Pose_t startPos , Pose_t endPos ,\
						   float percent1 , float percent2 , float posture1 , float posture2)
{
	Pose_t inter1 , inter2 = {0};
	path[0] = startPos;
	path[3] = endPos;
	
	inter1.point.x = startPos.point.x + percent1 * (endPos.point.x - startPos.point.x);
	inter1.point.y = startPos.point.y + percent1 * (endPos.point.y - startPos.point.y);
	inter1.direction = posture1;
	inter1.vel = MAX_PLAN_VEL;
	
	inter2.point.x = startPos.point.x + percent2 * (endPos.point.x - startPos.point.x);
	inter2.point.y = startPos.point.y + percent2 * (endPos.point.y - startPos.point.y);
	inter2.direction = posture2;
	inter2.vel = MAX_PLAN_VEL;	
	
	path[1] = inter1;
	path[2] = inter2;

}

void PathInit(uint8_t courtId)
{
	//默认轨迹为蓝场轨迹，如果为红场将轨迹X坐标和姿态角度取负
	if(courtId==RED_COURT)
	{
		for(uint8_t i=0;i<GET_FIRST_SHAGAI_PATH_NUM;i++)
		{
			getFirstShagaiPath[i].point.x = -(getFirstShagaiPath[i].point.x);
			getFirstShagaiPath[i].direction = -(getFirstShagaiPath[i].direction);			
		}
		for(uint8_t i=0;i<GO2_TZ_PATH_NUM;i++)
		{
			go2TZPath[i].point.x = -(go2TZPath[i].point.x);
			go2TZPath[i].direction = -(go2TZPath[i].direction);			
		}
		for(uint8_t i=0;i<THROW_FIRST_SHAGAI_PATH_NUM;i++)
		{
			throwFirstShagaiPath[i].point.x = -(throwFirstShagaiPath[i].point.x);
			throwFirstShagaiPath[i].direction = -(throwFirstShagaiPath[i].direction);			
		}
		for(uint8_t i=0;i<GET_2ND_SHAGAI_PATH_NUM;i++)
		{
			get2ndShagaiPath[i].point.x = -(get2ndShagaiPath[i].point.x);
			get2ndShagaiPath[i].direction = -(get2ndShagaiPath[i].direction);			
		}
		for(uint8_t i=0;i<THROW_2ND_SHAGAI_PATH_NUM;i++)
		{
			throw2ndShagaiPath[i].point.x = -(throw2ndShagaiPath[i].point.x);
			throw2ndShagaiPath[i].direction = -(throw2ndShagaiPath[i].direction);			
		}
		for(uint8_t i=0;i<GET_3RD_SHAGAI_PATH_NUM;i++)
		{
			get3rdShagaiPath[i].point.x = -(get3rdShagaiPath[i].point.x);
			get3rdShagaiPath[i].direction = -(get3rdShagaiPath[i].direction);			
		}
		for(uint8_t i=0;i<THROW_3RD_SHAGAI_PATH_NUM;i++)
		{
			throw3rdShagaiPath[i].point.x = -(throw3rdShagaiPath[i].point.x);
			throw3rdShagaiPath[i].direction = -(throw3rdShagaiPath[i].direction);			
		}		
	}
}

Pose_t testPath[6]=
{
{	0.f	,	0.f	,	0.0f	,	0.0f	},
{	-500.0f	,	0.f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1000.0f	,	0.f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1500.0f	,	0.f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2000.0f	,	0.f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2500.0f	,	0.f	,	0.0f	,	0.0f	}
};
//从出发区出发取第一个兽骨轨迹 校正 斜靠墙 向外移动15mm 车框全部在出发区内 107个点 
Pose_t getFirstShagaiPath[GET_FIRST_SHAGAI_PATH_NUM]=
{
{0.0f,	0.0f,	0.0f,	0.0f},
{	-259.51f	,	295.43f	,	0.0f	,	MAX_PLAN_VEL	},
{	-519.02f	,	590.87f	,	0.0f	,	MAX_PLAN_VEL	},
{	-778.53f	,	886.30f	,	0.0f	,	MAX_PLAN_VEL	},
{	-863.98f	,	969.71f	,	0.0f	,	MAX_PLAN_VEL	},
{	-961.19f	,	1039.06f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1067.86f	,	1092.73f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1181.49f	,	1129.45f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1299.40f	,	1148.37f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1418.81f	,	1149.02f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1536.92f	,	1131.40f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1650.94f	,	1095.93f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1758.19f	,	1043.43f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1856.16f	,	975.15f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1942.52f	,	892.68f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2015.26f	,	797.98f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2072.66f	,	693.26f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2113.36f	,	581.00f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2164.73f	,	446.12f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2240.11f	,	323.05f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2336.93f	,	216.01f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2451.83f	,	128.68f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2580.89f	,	64.06f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2719.65f	,	24.38f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2863.36f	,	11.00f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3007.07f	,	24.38f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3145.83f	,	64.06f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3274.89f	,	128.68f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3389.80f	,	216.01f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3486.61f	,	323.05f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3561.99f	,	446.12f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3613.36f	,	581.00f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3655.97f	,	698.82f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3715.94f	,	808.82f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3791.86f	,	908.47f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3882.00f	,	995.48f	,	0.0f	,	MAX_PLAN_VEL	},
{	-3984.28f	,	1067.84f	,	0.0f	,	MAX_PLAN_VEL	},
{	-4096.33f	,	1123.87f	,	0.0f	,	MAX_PLAN_VEL	},
{	-4215.58f	,	1162.30f	,	0.0f	,	MAX_PLAN_VEL	},
{	-4339.27f	,	1182.22f	,	0.0f	,	MAX_PLAN_VEL	},
{	-4464.55f	,	1183.18f	,	0.0f	,	MAX_PLAN_VEL	},
{	-4588.53f	,	1165.17f	,	0.0f	,	MAX_PLAN_VEL	},
{	-4708.35f	,	1128.58f	,	0.0f	,	MAX_PLAN_VEL	},
{	-4821.25f	,	1074.28f	,	0.0f	,	MAX_PLAN_VEL	},
{	-4924.63f	,	1003.51f	,	0.0f	,	MAX_PLAN_VEL	},
{	-5016.10f	,	917.89f	,	0.0f	,	MAX_PLAN_VEL	},
{	-5101.07f	,	835.64f	,	0.0f	,	MAX_PLAN_VEL	},
{	-5195.16f	,	763.99f	,	0.0f	,	MAX_PLAN_VEL	},
{	-5297.05f	,	703.95f	,	0.0f	,	MAX_PLAN_VEL	},
{	-5405.32f	,	656.35f	,	0.0f	,	MAX_PLAN_VEL	},
{	-5518.44f	,	621.88f	,	0.0f	,	MAX_PLAN_VEL	},
{	-5634.85f	,	600.99f	,	0.0f	,	MAX_PLAN_VEL	},
{	-5752.90f	,	594.00f	,	0.0f	,	MAX_PLAN_VEL	},
{	-6317.43f	,	594.00f	,	0.0f	,	MAX_PLAN_VEL	},
{	-6881.96f	,	594.00f	,	0.0f	,	MAX_PLAN_VEL	},
{	-7446.49f	,	594.00f	,	0.0f	,	MAX_PLAN_VEL	},
{	-7569.96f	,	603.23f	,	0.0f	,	MAX_PLAN_VEL	},
{	-7690.68f	,	630.73f	,	0.0f	,	MAX_PLAN_VEL	},
{	-7805.97f	,	675.88f	,	0.0f	,	MAX_PLAN_VEL	},
{	-7913.26f	,	737.68f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8010.16f	,	814.76f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8094.52f	,	905.39f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8164.46f	,	1007.56f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8218.42f	,	1118.99f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8255.20f	,	1237.22f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8273.99f	,	1359.60f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8274.36f	,	1483.41f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8256.31f	,	1605.91f	,	0.0f	,	MAX_PLAN_VEL	},
{	-8092.04f	,	2337.21f	,	0.0f	,	MAX_PLAN_VEL	},
{	-7927.77f	,	3068.52f	,	0.0f	,	MAX_PLAN_VEL	},
{	-7763.50f	,	3799.83f	,	0.0f	,	(0.4f*MAX_PLAN_VEL)	},
{	-7742.72f	,	3905.56f	,	0.0f	,	(0.4f*MAX_PLAN_VEL)	},
{	-7727.65f	,	4012.38f	,	0.0f	,	(0.4f*MAX_PLAN_VEL)	},
{	-7718.36f	,	4120.00f	,	0.0f	,	(0.4f*MAX_PLAN_VEL)	},
{	-7714.90f	,	4228.10f	,	0.0f	,	(0.4f*MAX_PLAN_VEL)	},
{	-7712.93f	,	4618.73f	,	0.0f	,	(0.35f*MAX_PLAN_VEL)	},
{	-7710.97f	,	5009.36f	,	0.0f	,	(0.35f*MAX_PLAN_VEL)	},
{	-7709.00f	,	5400.00f	,	0.0f	,	0.0f	}
};
//更改为最终角为77.81度
Pose_t go2TZPath[GO2_TZ_PATH_NUM]=
{
{	-7719.00f	,	5400.00f	,	0.00f	,	0.0f	},
{	-7719.00f	,	5200.00f	,	0.00f	,	MAX_PLAN_VEL	},
{	-7747.89f	,	5044.30f	,	-3.88f	,	MAX_PLAN_VEL	},
{	-7775.20f	,	4890.06f	,	-7.77f	,	MAX_PLAN_VEL	},
{	-7800.84f	,	4737.18f	,	-11.65f	,	MAX_PLAN_VEL	},
{	-7824.73f	,	4585.55f	,	-16.14f	,	MAX_PLAN_VEL	},
{	-7846.79f	,	4435.06f	,	-20.89f	,	MAX_PLAN_VEL	},
{	-7866.95f	,	4285.58f	,	-25.65f	,	MAX_PLAN_VEL	},
{	-7885.15f	,	4136.98f	,	-30.40f	,	MAX_PLAN_VEL	},
{	-7901.34f	,	3989.15f	,	-35.16f	,	MAX_PLAN_VEL	},
{	-7915.47f	,	3841.94f	,	-39.91f	,	MAX_PLAN_VEL	},
{	-7927.51f	,	3695.23f	,	-44.67f	,	MAX_PLAN_VEL	},
{	-7937.42f	,	3548.88f	,	-49.42f	,	MAX_PLAN_VEL	},
{	-7945.20f	,	3402.76f	,	-54.18f	,	MAX_PLAN_VEL	},
{	-7950.83f	,	3256.71f	,	-58.93f	,	MAX_PLAN_VEL	},
{	-7954.30f	,	3110.62f	,	-63.69f	,	MAX_PLAN_VEL	},
{	-7955.63f	,	2964.33f	,	-68.44f	,	MAX_PLAN_VEL	},
{	-7954.84f	,	2817.71f	,	-73.20f	,	MAX_PLAN_VEL	},
{	-7951.95f	,	2670.63f	,	-77.81f	,	MAX_PLAN_VEL	},
{	-7946.98f	,	2522.94f	,	-77.81f	,	MAX_PLAN_VEL	},
{	-7940.00f	,	2374.53f	,	-77.81f	,	MAX_PLAN_VEL	},
{	-7931.00f	,	2225.34f	,	-77.81f	,	0.0f	}
};
//0516前所用轨迹
//Pose_t go2TZPath[GO2_TZ_PATH_NUM]=
//{
//{	-7719.00f	,	5400.00f	,	0.00f	,	0.0f	},
//{	-7719.00f	,	5200.00f	,	0.00f	,	MAX_PLAN_VEL	},
//{	-7747.89f	,	5044.30f	,	-3.88f	,	MAX_PLAN_VEL	},
//{	-7775.20f	,	4890.06f	,	-7.77f	,	MAX_PLAN_VEL	},
//{	-7800.84f	,	4737.18f	,	-11.65f	,	MAX_PLAN_VEL	},
//{	-7824.73f	,	4585.55f	,	-15.54f	,	MAX_PLAN_VEL	},
//{	-7846.79f	,	4435.06f	,	-19.42f	,	MAX_PLAN_VEL	},
//{	-7866.95f	,	4285.58f	,	-23.31f	,	MAX_PLAN_VEL	},
//{	-7885.15f	,	4136.98f	,	-27.19f	,	MAX_PLAN_VEL	},
//{	-7901.34f	,	3989.15f	,	-31.08f	,	MAX_PLAN_VEL	},
//{	-7915.47f	,	3841.94f	,	-34.96f	,	MAX_PLAN_VEL	},
//{	-7927.51f	,	3695.23f	,	-38.85f	,	MAX_PLAN_VEL	},
//{	-7937.42f	,	3548.88f	,	-42.73f	,	MAX_PLAN_VEL	},
//{	-7945.20f	,	3402.76f	,	-46.62f	,	MAX_PLAN_VEL	},
//{	-7950.83f	,	3256.71f	,	-50.50f	,	MAX_PLAN_VEL	},
//{	-7954.30f	,	3110.62f	,	-54.39f	,	MAX_PLAN_VEL	},
//{	-7955.63f	,	2964.33f	,	-58.27f	,	MAX_PLAN_VEL	},
//{	-7954.84f	,	2817.71f	,	-62.16f	,	MAX_PLAN_VEL	},
//{	-7951.95f	,	2670.63f	,	-66.04f	,	MAX_PLAN_VEL	},
//{	-7946.98f	,	2522.94f	,	-69.93f	,	MAX_PLAN_VEL	},
//{	-7940.00f	,	2374.53f	,	-73.81f	,	MAX_PLAN_VEL	},
//{	-7931.00f	,	2225.34f	,	-73.81f	,	0.0f	}
//};
//原轨迹
//Pose_t go2TZPath[GO2_TZ_PATH_NUM]=
//{
//{	-7921.00f 	,	5152.00f 	,	0.00f 	,	0.0f	},
//{	-7921.00f 	,	3770.50f 	,	-3.90f 	,	MAX_PLAN_VEL	},
//{	-7921.00f 	,	2792.50f 	,	-50.70f 	,	MAX_PLAN_VEL	},
//{	-7921.00f 	,	2221.34f 	,	-78.00f 	,	0.0f	}
//};
//0516第一版测试轨迹
Pose_t throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM]=
{
{	-7931.00f	,	2225.34f	,	-77.81f	,	0.0f	},
//{	-7706.19f	,	2273.90f	,	-77.81f	,	MAX_PLAN_VEL	},
//{	-7481.37f	,	2322.46f	,	-77.81f	,	MAX_PLAN_VEL	},
//{	-7256.56f	,	2371.03f	,	-77.81f	,	MAX_PLAN_VEL	},
//{	-7031.75f	,	2419.59f	,	-77.81f	,	MAX_PLAN_VEL	},
//{	-6806.93f	,	2468.15f	,	-77.81f	,	MAX_PLAN_VEL	},
{	-6582.12f	,	2516.71f	,	-77.81f	,	MAX_PLAN_VEL	},
{	-5233.24f	,	2808.08f	,	-77.81f	,	MAX_PLAN_VEL	},
{	-3884.36f	,	3099.45f	,	-77.81f	,	0.0f	}
};
//0516之前所用轨迹
//Pose_t throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM]=
//{
////{	-7931.00	,	2225.34	,	-73.81f	,	0.0f	},
//{	-7795.63	,	2252.82	,	-73.81f	,	0.0f	},
//{	-6577.30	,	2500.09	,	-73.81f	,	MAX_PLAN_VEL	},
//{	-5223.59	,	2774.85	,	-73.81f	,	MAX_PLAN_VEL	},
//{	-3869.89	,	3049.60	,	-73.81f	,	0.0f	}
//};
//4.18日之前所用轨迹
//Pose_t throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM]=
//{
////{	-7931.00f	,	2225.34f	,	-73.81f	,	0.0f	},
//{	-7829.06f	,	2246.03f	,	-73.81f	,	0.0f	},
//{	-7715.15f	,	2269.15f	,	-73.81f	,	MAX_PLAN_VEL	},
//{	-6339.09f	,	2548.44f	,	-73.81f	,	MAX_PLAN_VEL	},
//{	-5428.21f	,	2733.31f	,	-73.81f	,	MAX_PLAN_VEL	},
//{	-4318.77f	,	2958.49f	,	-73.81f	,	MAX_PLAN_VEL	},
//{	-3869.89f	,	3049.60f	,	-73.81f	,	0.0f	}
//};
////原轨迹
//Pose_t throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM]=
//{
//{	-7921.00f	,	2221.34f	,	-78.0f	,	0.0f	},
//{	-7004.76f	,	2416.09f	,	-78.0f	,	MAX_PLAN_VEL	},
//{	-5071.60f	,	2827.00f	,	-78.0f	,	MAX_PLAN_VEL	},
//{	-3819.49f	,	3093.14f	,	-78.0f	,	0.0f	}
//};

Pose_t get2ndShagaiPath[GET_2ND_SHAGAI_PATH_NUM] =
{
{	-3870.04f	,	3050.00f	,	-73.81f	,	0.0f	},
{	-3917.50f	,	2986.49f	,	-65.81f	,	MAX_PLAN_VEL	},
{	-3967.43f	,	2924.91f	,	-53.81f	,	MAX_PLAN_VEL	},
{	-4019.76f	,	2865.35f	,	-41.81f	,	MAX_PLAN_VEL	},
{	-4074.40f	,	2807.91f	,	-29.81f	,	MAX_PLAN_VEL	},
{	-4131.26f	,	2752.67f	,	-17.81f	,	MAX_PLAN_VEL	},
{	-4190.27f	,	2699.71f	,	-5.81f	,	MAX_PLAN_VEL	},
{	-4251.32f	,	2649.14f	,	6.19f		,	MAX_PLAN_VEL	},
{	-4314.33f	,	2601.01f	,	18.19f	,	MAX_PLAN_VEL	},
{	-4379.19f	,	2555.42f	,	30.19f	,	MAX_PLAN_VEL	},
{	-4445.80f	,	2512.42f	,	42.19f	,	MAX_PLAN_VEL	},
{	-4514.05f	,	2472.08f	,	54.19f	,	MAX_PLAN_VEL	},
{	-4583.85f	,	2434.48f	,	66.19f	,	MAX_PLAN_VEL	},
{	-4655.08f	,	2399.67f	,	78.19f	,	MAX_PLAN_VEL	},
{	-4727.63f	,	2367.69f	,	86.00f	,	MAX_PLAN_VEL	},
{	-4801.38f	,	2338.61f	,	90.00f	,	MAX_PLAN_VEL	},
{	-4876.23f	,	2312.47f	,	90.00f	,	MAX_PLAN_VEL	},
{	-4952.05f	,	2289.31f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5028.73f	,	2269.16f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5106.15f	,	2252.05f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5184.18f	,	2238.02f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5262.70f	,	2227.09f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5341.60f	,	2219.27f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5420.74f	,	2214.57f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5500.00f	,	2213.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-6377.50f	,	2213.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-7255.00f	,	2213.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-8132.50f	,	2213.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-9010.00f	,	2213.00f	,	90.00f	,	0.0f	}
};															

//0423之前所用轨迹
//Pose_t get2ndShagaiPath[GET_2ND_SHAGAI_PATH_NUM] =
//{
//{	-3869.89f	,	3049.60f	,	-73.81f	,	0.0f	},
//{	-3948.10f	,	2996.93f	,	-66.01f	,	MAX_PLAN_VEL	},
//{	-4027.33f	,	2945.82f	,	-58.21f	,	MAX_PLAN_VEL	},
//{	-4107.57f	,	2896.30f	,	-50.41f	,	MAX_PLAN_VEL	},
//{	-4188.78f	,	2848.38f	,	-42.61f	,	MAX_PLAN_VEL	},
//{	-4270.92f	,	2802.08f	,	-27.01f	,	MAX_PLAN_VEL	},
//{	-4437.81f	,	2714.45f	,	-19.21f	,	MAX_PLAN_VEL	},
//{	-4522.63f	,	2673.10f	,	-11.41f	,	MAX_PLAN_VEL	},
//{	-4608.18f	,	2633.45f	,	-3.61f	,	MAX_PLAN_VEL	},
//{	-4694.51f	,	2595.52f	,	4.19f		,	MAX_PLAN_VEL	},
//{	-4781.57f	,	2559.31f	,	12.00f	,	MAX_PLAN_VEL	},
//{	-4869.34f	,	2524.83f	,	19.80f	,	MAX_PLAN_VEL	},
//{	-4957.76f	,	2492.11f	,	27.60f	,	MAX_PLAN_VEL	},
//{	-5046.82f	,	2461.15f	,	35.40f	,	MAX_PLAN_VEL	},
//{	-5136.48f	,	2431.96f	,	43.20f	,	MAX_PLAN_VEL	},
//{	-5226.69f	,	2404.55f	,	51.00f	,	MAX_PLAN_VEL	},
//{	-5317.44f	,	2378.95f	,	58.80f	,	MAX_PLAN_VEL	},
//{	-5408.68f	,	2355.14f	,	66.60f	,	MAX_PLAN_VEL	},
//{	-5500.37f	,	2333.16f	,	74.40f	,	MAX_PLAN_VEL	},
//{	-5592.48f	,	2313.00f	,	82.20f	,	MAX_PLAN_VEL	},
//{	-5641.48f	,	2303.05f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5715.99f	,	2288.95f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5883.49f	,	2261.71f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5969.10f	,	2250.15f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6179.99f	,	2228.38f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6512.00f	,	2213.26f	,	90.00f	,	MAX_PLAN_VEL	},
////{	-6562.00f	,	2213.26f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7426.75f	,	2213.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8314.67f	,	2213.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-9006.00f	,	2213.00f	,	90.00f	,	0.0f	}
//};
//原轨迹
//Pose_t get2ndShagaiPath[GET_2ND_SHAGAI_PATH_NUM]=
//{
////{	-3819.49f	,	3093.14f	,	-78.00f	,	0.0f	},
////{	-3819.99f	,	3095.50f	,	-69.60f	,	MAX_PLAN_VEL	},
//{	-3891.70f	,	3036.64f	,	-61.20f	,	0.0f	},
//{	-3965.17f	,	2981.79f	,	-52.80f	,	MAX_PLAN_VEL	},
//{	-4039.86f	,	2928.61f	,	-44.40f	,	MAX_PLAN_VEL	},
//{	-4115.73f	,	2877.14f	,	-36.00f	,	MAX_PLAN_VEL	},
//{	-4192.75f	,	2827.40f	,	-27.60f	,	MAX_PLAN_VEL	},
//{	-4270.87f	,	2779.41f	,	-19.20f	,	MAX_PLAN_VEL	},
//{	-4350.06f	,	2733.20f	,	-10.80f	,	MAX_PLAN_VEL	},
//{	-4430.28f	,	2688.80f	,	-2.40f	,	MAX_PLAN_VEL	},
//{	-4511.48f	,	2646.23f	,	6.00f	,	MAX_PLAN_VEL	},
//{	-4593.62f	,	2605.50f	,	14.40f	,	MAX_PLAN_VEL	},
//{	-4676.67f	,	2566.64f	,	22.80f	,	MAX_PLAN_VEL	},
//{	-4760.57f	,	2529.67f	,	31.20f	,	MAX_PLAN_VEL	},
//{	-4845.29f	,	2494.61f	,	39.60f	,	MAX_PLAN_VEL	},
//{	-4930.78f	,	2461.48f	,	48.00f	,	MAX_PLAN_VEL	},
//{	-5016.99f	,	2430.29f	,	56.40f	,	MAX_PLAN_VEL	},
//{	-5103.89f	,	2401.05f	,	64.80f	,	MAX_PLAN_VEL	},
//{	-5191.43f	,	2373.80f	,	73.20f	,	MAX_PLAN_VEL	},
//{	-5279.57f	,	2348.53f	,	81.60f	,	MAX_PLAN_VEL	},
//{	-5368.25f	,	2325.26f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5457.44f	,	2304.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5567.74f	,	2280.64f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5678.91f	,	2261.90f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5790.07f	,	2243.16f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5902.11f	,	2230.64f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6014.15f	,	2218.11f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6126.71f	,	2211.84f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6352.00f	,	2204.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6704.00f	,	2204.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7079.40f	,	2204.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8199.94f	,	2204.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-9000.00f	,	2204.00f	,	90.00f	,	0.0f	}	
//};
Pose_t throw2ndShagaiPath[THROW_2ND_SHAGAI_PATH_NUM]=
{
{	-9006.00f	,	2212.99f	,	90.00f	,	0.0f	},
{	-8808.61f	,	2245.15f	,	90.00f	,	MAX_PLAN_VEL	},
{	-8660.36f	,	2269.30f	,	81.81f	,	MAX_PLAN_VEL	},
{	-8512.13f	,	2293.44f	,	73.62f	,	MAX_PLAN_VEL	},
{	-8363.90f	,	2317.59f	,	65.43f	,	MAX_PLAN_VEL	},
{	-8215.66f	,	2341.73f	,	57.24f	,	MAX_PLAN_VEL	},
{	-8067.42f	,	2365.88f	,	49.05f	,	MAX_PLAN_VEL	},
{	-7919.19f	,	2390.03f	,	40.86f	,	MAX_PLAN_VEL	},
{	-7770.95f	,	2414.17f	,	32.67f	,	MAX_PLAN_VEL	},
{	-7622.46f	,	2438.22f	,	24.48f	,	MAX_PLAN_VEL	},
{	-7474.48f	,	2462.46f	,	16.29f	,	MAX_PLAN_VEL	},
{	-7326.24f	,	2486.61f	,	8.10f	,	MAX_PLAN_VEL	},
{	-7178.00f	,	2510.75f	,	-0.10f	,	MAX_PLAN_VEL	},
{	-7029.76f	,	2534.90f	,	-8.29f	,	MAX_PLAN_VEL	},
{	-6881.53f	,	2559.05f	,	-16.48f	,	MAX_PLAN_VEL	},
{	-6733.30f	,	2583.19f	,	-24.67f	,	MAX_PLAN_VEL	},
{	-6585.06f	,	2607.33f	,	-32.86f	,	MAX_PLAN_VEL	},
{	-6436.82f	,	2631.48f	,	-41.05f	,	MAX_PLAN_VEL	},
{	-6288.58f	,	2655.63f	,	-49.24f	,	MAX_PLAN_VEL	},
{	-6140.35f	,	2679.77f	,	-57.43f	,	MAX_PLAN_VEL	},
{	-5992.11f	,	2703.92f	,	-65.62f	,	MAX_PLAN_VEL	},
{	-5843.87f	,	2728.06f	,	-73.81f	,	MAX_PLAN_VEL	},
{	-5359.79f	,	2806.91f	,	-73.81f	,	MAX_PLAN_VEL	},
{	-4500.14f	,	2946.94f	,	-73.81f	,	MAX_PLAN_VEL	},
{	-3869.89f	,	3049.60f	,	-73.81f	,	0.0f	},
};
//原轨迹
//Pose_t throw2ndShagaiPath[THROW_2ND_SHAGAI_PATH_NUM]=
//{
////{	-9000.00f	,	2204.00f	,	90.00f	,	0.0f	},
//{	-8802.88f	,	2237.83f	,	90.00f	,	0.0f	},
//{	-8652.27f	,	2263.68f	,	81.60f	,	MAX_PLAN_VEL	},
//{	-8501.66f	,	2289.53f	,	73.20f	,	MAX_PLAN_VEL	},
//{	-8351.05f	,	2315.38f	,	64.80f	,	MAX_PLAN_VEL	},
//{	-8200.44f	,	2341.23f	,	56.40f	,	MAX_PLAN_VEL	},
//{	-8049.83f	,	2367.08f	,	48.00f	,	MAX_PLAN_VEL	},
//{	-7899.22f	,	2392.93f	,	39.60f	,	MAX_PLAN_VEL	},
//{	-7748.61f	,	2418.78f	,	31.20f	,	MAX_PLAN_VEL	},
//{	-7598.00f	,	2444.63f	,	22.80f	,	MAX_PLAN_VEL	},
//{	-7447.39f	,	2470.48f	,	14.40f	,	MAX_PLAN_VEL	},
//{	-7296.78f	,	2496.33f	,	6.00f	,	MAX_PLAN_VEL	},
//{	-7146.17f	,	2522.18f	,	-2.40f	,	MAX_PLAN_VEL	},
//{	-6995.56f	,	2548.03f	,	-10.80f	,	MAX_PLAN_VEL	},
//{	-6844.95f	,	2573.88f	,	-19.20f	,	MAX_PLAN_VEL	},
//{	-6694.33f	,	2599.73f	,	-27.60f	,	MAX_PLAN_VEL	},
//{	-6543.72f	,	2625.58f	,	-36.00f	,	MAX_PLAN_VEL	},
//{	-6393.11f	,	2651.43f	,	-44.40f	,	MAX_PLAN_VEL	},
//{	-6242.50f	,	2677.28f	,	-52.80f	,	MAX_PLAN_VEL	},
//{	-6091.89f	,	2703.13f	,	-61.20f	,	MAX_PLAN_VEL	},
//{	-5941.28f	,	2728.98f	,	-69.60f	,	MAX_PLAN_VEL	},
//{	-5790.67f	,	2754.83f	,	-78.00f	,	MAX_PLAN_VEL	},
//{	-5159.95f	,	2863.08f	,	-78.00f	,	MAX_PLAN_VEL	},
//{	-4547.20f	,	2968.24f	,	-78.00f	,	MAX_PLAN_VEL	},
//{	-3819.49f	,	3093.14f	,	-78.00f	,	0.0f	}	
//};
//一开始48最后96333
Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM] = 
{
//{	-3870.04f	,	3050.00f	,	-73.81f	,	0.0f	},
{	-3870.04f	,	3050.00f	,	-77.81f	,	0.0f	},
{	-3942.79f	,	3077.69f	,	-81.81f	,	MAX_PLAN_VEL	},
{	-4015.89f	,	3104.42f	,	-89.81f	,	MAX_PLAN_VEL	},
{	-4089.34f	,	3130.18f	,	-101.81f	,	MAX_PLAN_VEL	},
{	-4163.13f	,	3154.97f	,	-113.81f	,	MAX_PLAN_VEL	},
{	-4237.23f	,	3178.77f	,	-125.81f	,	MAX_PLAN_VEL	},
{	-4311.64f	,	3201.59f	,	-137.81f	,	MAX_PLAN_VEL	},
{	-4386.36f	,	3223.44f	,	-149.81f	,	MAX_PLAN_VEL	},
{	-4461.35f	,	3244.28f	,	-161.81f	,	MAX_PLAN_VEL	},
{	-4536.61f	,	3264.14f	,	-173.81f	,	MAX_PLAN_VEL	},
{	-4612.13f	,	3283.00f	,	174.19f	,	MAX_PLAN_VEL	},
{	-4687.89f	,	3300.85f	,	162.19f	,	MAX_PLAN_VEL	},
{	-4763.88f	,	3317.71f	,	150.19f	,	MAX_PLAN_VEL	},
{	-4840.09f	,	3333.56f	,	138.19f	,	MAX_PLAN_VEL	},
{	-4916.50f	,	3348.39f	,	126.19f	,	MAX_PLAN_VEL	},
{	-4993.10f	,	3362.21f	,	114.00f	,	MAX_PLAN_VEL	},
{	-5069.87f	,	3375.03f	,	105.00f	,	MAX_PLAN_VEL	},
{	-5146.81f	,	3386.82f	,	99.00f	,	MAX_PLAN_VEL	},
{	-5223.89f	,	3397.60f	,	96.00f	,	MAX_PLAN_VEL	},
{	-5301.12f	,	3407.35f	,	93.00f	,	MAX_PLAN_VEL	},
{	-5378.46f	,	3416.08f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5455.92f	,	3423.79f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5533.46f	,	3430.48f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5611.09f	,	3436.14f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5688.79f	,	3440.77f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5766.55f	,	3444.37f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5844.34f	,	3446.94f	,	90.00f	,	MAX_PLAN_VEL	},
{	-5922.16f	,	3448.49f	,	90.00f	,	MAX_PLAN_VEL	},
{	-6000.00f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-6752.50f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-7505.00f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-8257.50f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-9010.00f	, 	3449.00f	,	90.00f	,	0.0f	}
};
////最后96333
//Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM] = 
//{
////{	-3870.04f	,	3050.00f	,	-73.81f	,	0.0f	},
//{	-3870.04f	,	3050.00f	,	-77.81f	,	0.0f	},
//{	-3942.79f	,	3077.69f	,	-89.81f	,	MAX_PLAN_VEL	},
//{	-4015.89f	,	3104.42f	,	-101.81f	,	MAX_PLAN_VEL	},
//{	-4089.34f	,	3130.18f	,	-113.81f	,	MAX_PLAN_VEL	},
//{	-4163.13f	,	3154.97f	,	-125.81f	,	MAX_PLAN_VEL	},
//{	-4237.23f	,	3178.77f	,	-137.81f	,	MAX_PLAN_VEL	},
//{	-4311.64f	,	3201.59f	,	-149.81f	,	MAX_PLAN_VEL	},
//{	-4386.36f	,	3223.44f	,	-161.81f	,	MAX_PLAN_VEL	},
//{	-4461.35f	,	3244.28f	,	-173.81f	,	MAX_PLAN_VEL	},
//{	-4536.61f	,	3264.14f	,	174.81f	,	MAX_PLAN_VEL	},
//{	-4612.13f	,	3283.00f	,	162.19f	,	MAX_PLAN_VEL	},
//{	-4687.89f	,	3300.85f	,	150.19f	,	MAX_PLAN_VEL	},
//{	-4763.88f	,	3317.71f	,	138.19f	,	MAX_PLAN_VEL	},
//{	-4840.09f	,	3333.56f	,	126.19f	,	MAX_PLAN_VEL	},
//{	-4916.50f	,	3348.39f	,	114.19f	,	MAX_PLAN_VEL	},
//{	-4993.10f	,	3362.21f	,	105.00f	,	MAX_PLAN_VEL	},
//{	-5069.87f	,	3375.03f	,	99.00f	,	MAX_PLAN_VEL	},
//{	-5146.81f	,	3386.82f	,	96.00f	,	MAX_PLAN_VEL	},
//{	-5223.89f	,	3397.60f	,	93.00f	,	MAX_PLAN_VEL	},
//{	-5301.12f	,	3407.35f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5378.46f	,	3416.08f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5455.92f	,	3423.79f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5533.46f	,	3430.48f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5611.09f	,	3436.14f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5688.79f	,	3440.77f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5766.55f	,	3444.37f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5844.34f	,	3446.94f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5922.16f	,	3448.49f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6000.00f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6752.50f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7505.00f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8257.50f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-9010.00f	, 	3449.00f	,	90.00f	,	0.0f	}
//};
//最后840
//Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM] = 
//{
////{	-3870.04f	,	3050.00f	,	-73.81f	,	0.0f	},
//{	-3870.04f	,	3050.00f	,	-77.81f	,	0.0f	},
//{	-3942.79f	,	3077.69f	,	-89.81f	,	MAX_PLAN_VEL	},
//{	-4015.89f	,	3104.42f	,	-101.81f	,	MAX_PLAN_VEL	},
//{	-4089.34f	,	3130.18f	,	-113.81f	,	MAX_PLAN_VEL	},
//{	-4163.13f	,	3154.97f	,	-125.81f	,	MAX_PLAN_VEL	},
//{	-4237.23f	,	3178.77f	,	-137.81f	,	MAX_PLAN_VEL	},
//{	-4311.64f	,	3201.59f	,	-149.81f	,	MAX_PLAN_VEL	},
//{	-4386.36f	,	3223.44f	,	-161.81f	,	MAX_PLAN_VEL	},
//{	-4461.35f	,	3244.28f	,	-173.81f	,	MAX_PLAN_VEL	},
//{	-4536.61f	,	3264.14f	,	174.81f	,	MAX_PLAN_VEL	},
//{	-4612.13f	,	3283.00f	,	162.19f	,	MAX_PLAN_VEL	},
//{	-4687.89f	,	3300.85f	,	150.19f	,	MAX_PLAN_VEL	},
//{	-4763.88f	,	3317.71f	,	138.19f	,	MAX_PLAN_VEL	},
//{	-4840.09f	,	3333.56f	,	126.19f	,	MAX_PLAN_VEL	},
//{	-4916.50f	,	3348.39f	,	114.19f	,	MAX_PLAN_VEL	},
//{	-4993.10f	,	3362.21f	,	102.19f	,	MAX_PLAN_VEL	},
//{	-5069.87f	,	3375.03f	,	94.19f	,	MAX_PLAN_VEL	},
//{	-5146.81f	,	3386.82f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5223.89f	,	3397.60f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5301.12f	,	3407.35f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5378.46f	,	3416.08f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5455.92f	,	3423.79f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5533.46f	,	3430.48f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5611.09f	,	3436.14f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5688.79f	,	3440.77f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5766.55f	,	3444.37f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5844.34f	,	3446.94f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5922.16f	,	3448.49f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6000.00f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6752.50f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7505.00f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8257.50f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-9010.00f	, 	3449.00f	,	90.00f	,	0.0f	}
//};
//提前一两个点转到90
//Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM] = 
//{
//{	-3870.04f	,	3050.00f	,	-77.81f	,	0.0f	},
//{	-3942.79f	,	3077.69f	,	-91.54f	,	MAX_PLAN_VEL	},
//{	-4015.89f	,	3104.42f	,	-105.27f	,	MAX_PLAN_VEL	},
//{	-4089.34f	,	3130.18f	,	-118.99f	,	MAX_PLAN_VEL	},
//{	-4163.13f	,	3154.97f	,	-132.72f	,	MAX_PLAN_VEL	},
//{	-4237.23f	,	3178.77f	,	-146.45f	,	MAX_PLAN_VEL	},
//{	-4311.64f	,	3201.59f	,	-160.18f	,	MAX_PLAN_VEL	},
//{	-4386.36f	,	3223.44f	,	-173.91f	,	MAX_PLAN_VEL	},
//{	-4461.35f	,	3244.28f	,	172.37f	,	MAX_PLAN_VEL	},
//{	-4536.61f	,	3264.14f	,	158.64f	,	MAX_PLAN_VEL	},
//{	-4612.13f	,	3283.00f	,	144.91f	,	MAX_PLAN_VEL	},
//{	-4687.89f	,	3300.85f	,	131.18f	,	MAX_PLAN_VEL	},
//{	-4763.88f	,	3317.71f	,	117.46f	,	MAX_PLAN_VEL	},
//{	-4840.09f	,	3333.56f	,	103.73f	,	MAX_PLAN_VEL	},
//{	-4916.50f	,	3348.39f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-4993.10f	,	3362.21f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5069.87f	,	3375.03f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5146.81f	,	3386.82f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5223.89f	,	3397.60f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5301.12f	,	3407.35f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5378.46f	,	3416.08f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5455.92f	,	3423.79f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5533.46f	,	3430.48f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5611.09f	,	3436.14f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5688.79f	,	3440.77f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5766.55f	,	3444.37f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5844.34f	,	3446.94f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5922.16f	,	3448.49f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6000.00f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6752.50f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7505.00f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8257.50f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-9010.00f	,	3449.00f	,	90.00f	,	0.0f	}
//};
//取第三个块轨迹老师2
//Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM] = 
//{
//{	-3870.04f	,	3050.00f	,	-77.81f	,	0.0f	},
//{	-3942.79f	,	3077.69f	,	-89.81f	,	MAX_PLAN_VEL	},
//{	-4015.89f	,	3104.42f	,	-101.81f	,	MAX_PLAN_VEL	},
//{	-4089.34f	,	3130.18f	,	-113.81f	,	MAX_PLAN_VEL	},
//{	-4163.13f	,	3154.97f	,	-125.81f	,	MAX_PLAN_VEL	},
//{	-4237.23f	,	3178.77f	,	-137.81f	,	MAX_PLAN_VEL	},
//{	-4311.64f	,	3201.59f	,	-149.81f	,	MAX_PLAN_VEL	},
//{	-4386.36f	,	3223.44f	,	-161.81f	,	MAX_PLAN_VEL	},
//{	-4461.35f	,	3244.28f	,	-173.81f	,	MAX_PLAN_VEL	},
//{	-4536.61f	,	3264.14f	,	174.19f	,	MAX_PLAN_VEL	},
//{	-4612.13f	,	3283.00f	,	162.19f	,	MAX_PLAN_VEL	},
//{	-4687.89f	,	3300.85f	,	150.19f	,	MAX_PLAN_VEL	},
//{	-4763.88f	,	3317.71f	,	138.19f	,	MAX_PLAN_VEL	},
//{	-4840.09f	,	3333.56f	,	126.19f	,	MAX_PLAN_VEL	},
//{	-4916.50f	,	3348.39f	,	114.19f	,	MAX_PLAN_VEL	},
//{	-4993.10f	,	3362.21f	,	102.19f	,	MAX_PLAN_VEL	},
//{	-5069.87f	,	3375.03f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5146.81f	,	3386.82f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5223.89f	,	3397.60f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5301.12f	,	3407.35f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5378.46f	,	3416.08f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5455.92f	,	3423.79f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5533.46f	,	3430.48f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5611.09f	,	3436.14f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5688.79f	,	3440.77f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5766.55f	,	3444.37f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5844.34f	,	3446.94f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5922.16f	,	3448.49f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6000.00f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6752.50f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7505.00f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8257.50f	, 	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-9010.00f	, 	3449.00f	,	90.00f	,	0.0f	}
//};
//4.18日之前所用轨迹
//Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM] = 
//{
////{	-3869.88f	,	3049.59f	,	180.00f	,	0.0f	},
//{	-3933.81f	,	3068.73f	,	175.50f	,	0.0f	},
//{	-3997.88f	,	3087.42f	,	171.00f	,	MAX_PLAN_VEL	},
//{	-4062.07f	,	3105.64f	,	166.50f	,	MAX_PLAN_VEL	},
//{	-4126.40f	,	3123.40f	,	162.00f	,	MAX_PLAN_VEL	},
//{	-4190.86f	,	3140.70f	,	157.50f	,	MAX_PLAN_VEL	},
//{	-4255.43f	,	3157.53f	,	153.00f	,	MAX_PLAN_VEL	},
//{	-4320.13f	,	3173.90f	,	148.50f	,	MAX_PLAN_VEL	},
//{	-4384.94f	,	3189.81f	,	144.00f	,	MAX_PLAN_VEL	},
//{	-4449.87f	,	3205.24f	,	139.50f	,	MAX_PLAN_VEL	},
//{	-4514.90f	,	3220.21f	,	135.00f	,	MAX_PLAN_VEL	},
//{	-4580.04f	,	3234.70f	,	130.50f	,	MAX_PLAN_VEL	},
//{	-4645.29f	,	3248.73f	,	126.00f	,	MAX_PLAN_VEL	},
//{	-4710.63f	,	3262.29f	,	121.50f	,	MAX_PLAN_VEL	},
//{	-4776.07f	,	3275.38f	,	117.00f	,	MAX_PLAN_VEL	},
//{	-4841.60f	,	3288.00f	,	112.50f	,	MAX_PLAN_VEL	},
//{	-4907.22f	,	3300.15f	,	108.00f	,	MAX_PLAN_VEL	},
//{	-4972.92f	,	3311.82f	,	103.50f	,	MAX_PLAN_VEL	},
//{	-5038.71f	,	3323.02f	,	99.00f	,	MAX_PLAN_VEL	},
//{	-5104.58f	,	3333.74f	,	94.50f	,	MAX_PLAN_VEL	},
//{	-5170.52f	,	3344.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5238.87f	,	3354.11f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5308.18f	,	3363.84f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5403.54f	,	3376.35f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5461.58f	,	3383.47f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5522.52f	,	3390.55f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5607.69f	,	3399.76f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5743.63f	,	3412.81f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5879.76f	,	3423.86f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6016.03f	,	3432.91f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6152.42f	,	3439.95f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6288.90f	,	3444.97f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6425.43f	,	3447.99f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6562.00f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7163.96f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7545.96f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8044.49f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8426.49f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-9006.00f	,	3449.00f	,	90.00f	,	0.0f	}

//};
	//原取3轨迹
//Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM] = 
//{
////{	-3869.89f	,	3049.59f	,	-73.81f	,	0.0f	},
////{	-3869.89f	,	3049.59f	,	10.79f	,	MAX_PLAN_VEL	},
////{	-3869.89f	,	3049.59f	,	95.40f	,	MAX_PLAN_VEL	},
////{	-3869.89f	,	3049.59f	,	180.00f	,	MAX_PLAN_VEL	},
////{	-3869.89f	,	3049.59f	,	264.60f	,	MAX_PLAN_VEL	},
////{	-3869.89f	,	3049.59f	,	349.21f	,	MAX_PLAN_VEL	},
////{	-3869.89f	,	3049.59f	,	90.00f	,	MAX_PLAN_VEL	},
////{	-4000.87f	,	3088.28f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-4132.40f	,	3125.03f	,	90.00f	,	0.0f	},
//{	-4264.46f	,	3159.85f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-4397.02f	,	3192.71f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-4530.04f	,	3223.62f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-4663.51f	,	3252.56f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-4797.39f	,	3279.54f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-4931.65f	,	3304.54f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5066.27f	,	3327.57f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5201.21f	,	3348.60f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5336.45f	,	3367.65f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5471.95f	,	3384.70f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5607.69f	,	3399.76f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5743.63f	,	3412.81f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-5879.76f	,	3423.86f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6016.03f	,	3432.91f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6152.42f	,	3439.95f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6288.90f	,	3444.97f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6425.43f	,	3447.99f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-6562.00f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7163.96f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-7545.96f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8044.49f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-8426.49f	,	3449.00f	,	90.00f	,	MAX_PLAN_VEL	},
//{	-9006.00f	,	3449.00f	,	90.00f	,	0.0f	}

//原轨迹
//Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM] = 
//{
////{	-3819.49f	,	3093.14f	,	90.00f	,	0.0f	}	,
//{	-3954.79f	,	3127.64f	,	90.00f	,	0.0f	}	,
//{	-4090.53f	,	3160.38f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-4226.68f	,	3191.37f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-4363.22f	,	3220.59f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-4500.12f	,	3248.04f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-4637.37f	,	3273.71f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-4774.94f	,	3297.61f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-4912.81f	,	3319.72f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-5050.95f	,	3340.04f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-5189.35f	,	3358.58f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-5327.97f	,	3375.32f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-5466.79f	,	3390.27f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-5605.80f	,	3403.42f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-5744.97f	,	3414.77f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-5884.27f	,	3424.31f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-6023.69f	,	3432.05f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-6163.19f	,	3437.99f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-6302.76f	,	3442.12f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-6442.37f	,	3444.44f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-6552.00f	,	3445.00f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-7061.98f	,	3445.00f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-7543.05f	,	3445.00f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-7886.39f	,	3445.00f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-8241.39f	,	3445.00f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-8593.39f	,	3445.00f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-9000.00f	,	3445.00f	,	90.00f	,	0.0f	}	,
//};
Pose_t throw3rdShagaiPath[THROW_3RD_SHAGAI_PATH_NUM] = 
{
{	-9006.00f	,	3449.00f	,	90.00f	,	0.0f	},
{	-8806.16f	,	3399.00f	,	90.00f	,	MAX_PLAN_VEL	},
{	-8637.53f	,	3359.52f	,	99.81f	,	MAX_PLAN_VEL	},
{	-8468.37f	,	3322.39f	,	109.62f	,	MAX_PLAN_VEL	},
{	-8298.71f	,	3287.59f	,	119.43f	,	MAX_PLAN_VEL	},
{	-8128.58f	,	3255.15f	,	129.24f	,	MAX_PLAN_VEL	},
{	-7958.02f	,	3225.06f	,	139.05f	,	MAX_PLAN_VEL	},
{	-7787.06f	,	3197.34f	,	148.86f	,	MAX_PLAN_VEL	},
{	-7615.73f	,	3171.99f	,	158.67f	,	MAX_PLAN_VEL	},
{	-7444.06f	,	3149.01f	,	168.48f	,	MAX_PLAN_VEL	},
{	-7272.10f	,	3128.41f	,	178.29f	,	MAX_PLAN_VEL	},
{	-7099.87f	,	3110.19f	,	-171.91f	,	MAX_PLAN_VEL	},
{	-6927.40f	,	3094.37f	,	-162.10f	,	MAX_PLAN_VEL	},
{	-6754.73f	,	3080.92f	,	-152.29f	,	MAX_PLAN_VEL	},
{	-6581.89f	,	3069.87f	,	-142.48f	,	MAX_PLAN_VEL	},
{	-6408.91f	,	3061.21f	,	-132.67f	,	MAX_PLAN_VEL	},
{	-6235.82f	,	3054.94f	,	-122.86f	,	MAX_PLAN_VEL	},
{	-6062.67f	,	3051.08f	,	-113.05f	,	MAX_PLAN_VEL	},
{	-5889.49f	,	3049.61f	,	-103.24f	,	MAX_PLAN_VEL	},
{	-5716.29f	,	3049.60f	,	-93.43f	,	MAX_PLAN_VEL	},
{	-5543.10f	,	3049.60f	,	-83.62f	,	MAX_PLAN_VEL	},
{	-5369.89f	,	3049.60f	,	-73.81f	,	MAX_PLAN_VEL	},
{	-5319.89f	,	3049.60f	,	-73.81f	,	MAX_PLAN_VEL	},
{	-5184.88f	,	3049.60f	,	-73.81f	,	MAX_PLAN_VEL	},
{	-4977.73f	,	3049.60f	,	-73.81f	,	MAX_PLAN_VEL	},
{	-4267.67f	,	3049.60f	,	-73.81f	,	MAX_PLAN_VEL	},
{	-3869.89f	,	3049.59f	,	-73.81f	,	0.0f	}
};
//原轨迹
//Pose_t throw3rdShagaiPath[THROW_3RD_SHAGAI_PATH_NUM] = 
//{
//{	-9000.00f	,	3445.00f	,	90.00f	,	0.0f	}	,
//{	-8800.00f	,	3361.51f	,	90.00f	,	MAX_PLAN_VEL	}	,
//{	-8658.44f	,	3309.90f	,	81.60f	,	MAX_PLAN_VEL	}	,
//{	-8515.04f	,	3263.67f	,	73.20f	,	MAX_PLAN_VEL	}	,
//{	-8370.00f	,	3222.87f	,	64.80f	,	MAX_PLAN_VEL	}	,
//{	-8223.52f	,	3187.56f	,	56.40f	,	MAX_PLAN_VEL	}	,
//{	-8075.82f	,	3157.79f	,	48.00f	,	MAX_PLAN_VEL	}	,
//{	-7927.10f	,	3133.61f	,	39.60f	,	MAX_PLAN_VEL	}	,
//{	-7777.58f	,	3115.05f	,	31.20f	,	MAX_PLAN_VEL	}	,
//{	-7627.46f	,	3102.12f	,	22.80f	,	MAX_PLAN_VEL	}	,
//{	-7476.96f	,	3094.87f	,	14.40f	,	MAX_PLAN_VEL	}	,
//{	-7326.30f	,	3093.14f	,	6.00f	,	MAX_PLAN_VEL	}	,
//{	-7175.62f	,	3093.14f	,	-2.40f	,	MAX_PLAN_VEL	}	,
//{	-7024.94f	,	3093.14f	,	-10.80f	,	MAX_PLAN_VEL	}	,
//{	-6874.26f	,	3093.14f	,	-19.20f	,	MAX_PLAN_VEL	}	,
//{	-6723.58f	,	3093.14f	,	-27.60f	,	MAX_PLAN_VEL	}	,
//{	-6572.90f	,	3093.14f	,	-36.00f	,	MAX_PLAN_VEL	}	,
//{	-6422.22f	,	3093.14f	,	-44.40f	,	MAX_PLAN_VEL	}	,
//{	-6271.53f	,	3093.14f	,	-52.80f	,	MAX_PLAN_VEL	}	,
//{	-6120.85f	,	3093.14f	,	-61.20f	,	MAX_PLAN_VEL	}	,
//{	-5970.17f	,	3093.14f	,	-69.60f	,	MAX_PLAN_VEL	}	,
//{	-5819.49f	,	3093.14f	,	-78.00f	,	MAX_PLAN_VEL	}	,
//{	-5222.40f	,	3093.14f	,	-78.00f	,	MAX_PLAN_VEL	}	,
//{	-4640.82f	,	3093.14f	,	-78.00f	,	MAX_PLAN_VEL	}	,
//{	-3819.49f	,	3093.14f	,	-78.00f	,	0.0f	}	
//};

