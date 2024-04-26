#ifndef MULTI_ROBOT_LAYER_H_
#define MULTI_ROBOT_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

// Queue size(最新のデータが欲しい場合は小さく，取りこぼしたくない場合は大きくする)
#define		ROS_QUEUE_SIZE				10

// GridMapコスト
#define     FREESPACE_COST_GRIDMAP		0
#define     UNKNOWN_COST_GRIDMAP		-1

// Costmapコスト
#define     FREESPACE_COST_COSTMAP2D    0
#define     INSCRIBED_COST_COSTMAP2D	253
#define		OBSTACLE_COST_COSTMAP2D		254
#define		UNKNOWN_COST_COSTMAP2D		255
#define     PLAN_COST_MAX				INSCRIBED_COST_COSTMAP2D

// ID,TYPE
#define     DEFAULT_ROBOT_ID        "turtlebot_01"
#define     DEFAULT_ROBOT_TYPE      "turtlebot"

namespace multi_robot_layer_namespace
{
	class MultiRobotLayer : public costmap_2d::Layer
	{
		public:
			/**
			* @brief        MultiRobotLayerクラスのコンストラクタ
			* @details      初期化を行う
			*/
			MultiRobotLayer();
			
			/**
			* @brief        MultiRobotLayerクラスのデストラクタ
			* @details      オブジェクトの破棄を行う
			*/
			~MultiRobotLayer();

			/**
			 * @brief       初期化処理の実装関数
			 * @param[in]   void
			 * @return      void
			 */
			virtual void onInitialize();

			/**
			 * @brief       更新範囲の境界値設定処理の実装関数
			 * @param[in]   double robot_x 
			 * @param[in]   double robot_y
			 * @param[in]   double robot_yaw
			 * @param[out]  double* min_x 更新範囲の最小の境界値(x方向)
			 * @param[out]  double* min_x 更新範囲の最小の境界値(y方向)
			 * @param[out]  double* max_x 更新範囲の最大の境界値(x方向)
			 * @param[out]  double* max_y 更新範囲の最大の境界値(y方向)
			 * @return      void
			 */
			virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
									  double* min_x, double* min_y, double* max_x,double* max_y);

			/**
			 * @brief       コストの更新処理の実装関数
			 * @param[in]   costmap_2d::Costmap2D& master_grid	マスターのコストマップ
			 * @param[in]   int min_i 
			 * @param[in]   int min_j	
			 * @param[out]  int max_i	
			 * @param[out]  int max_j	
			 * @return      void
			 */
			virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
									 int min_i, int min_j, int max_i, int max_j);

		private:
			typedef struct cell_index
			{
				unsigned int width_idx;
				unsigned int height_idx;
			}stCellIdx;

			ros::Subscriber sub_plan_costmap;		// 経路コストマップのサブスクライバ
			ros::Subscriber sub_my_plan_info;   	// 自己経路情報のサブスクライバ
    		costmap_2d::Costmap2D *buf_costmap;		// masterへ反映用のバッファ
			nav_msgs::OccupancyGrid plan_costmap;	// 他ロボット経路コストマップ
			dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

			std::string entityId; 							// getParamで取得する他ロボットの固有名詞
			std::vector<stCellIdx> *vct_plan_costmap_cell; 	// 経路コストマップの高コストのセルの幅及び高さを格納する

			unsigned char cost_trans_table[101]; 	// コストマップへ反映するコストの変換テーブル
			unsigned int plan_costmap_width;		// 経路コストマップのセルの幅
			unsigned int plan_costmap_height;		// 経路コストマップのセルの高さ
			unsigned int plan_costmap_width_min;	// 経路コストマップのセルの更新する幅の最小値
			unsigned int plan_costmap_width_max;	// 経路コストマップのセルの更新する幅の最大値
			unsigned int plan_costmap_height_min;	// 経路コストマップのセルの更新する高さの最小値
			unsigned int plan_costmap_height_max;	// 経路コストマップのセルの更新する高さの最大値
			double plan_costmap_resolution;			// 経路コストマップの解像度
			double plan_costmap_origin_x;			// グローバルフレーム内のマップのxの原点をメートル単位で指定する
			double plan_costmap_origin_y;			// グローバルフレーム内のマップのyの原点をメートル単位で指定する
			bool stay_costmap;						// 経路コストマップの反映フラグ
			bool update_costmap;					// 経路コストマップのアップデート監視フラグ
	

			/**
			 * @brief       コンフィグの読み込みコールバック関数
			 * @param[in]   costmap_2d::GenericPluginConfig &config
			 * @param[in]   uint32_t level
			 * @return      void
			 */
			void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

			/**
			 * @brief       通行区域制限用コストマップの受信処理
			 * @param[in]   const nav_msgs::OccupancyGridConstPtr& msg　経路用コストマップの受信
			 * @return      void
			 */
			void recvCostmap(const nav_msgs::OccupancyGridConstPtr& msg);

			/**
			 * @brief       コスト値の変換処理
			 * @param[in]   void
			 * @return      void
			 */
			void convertCost(void);

			/**
			 * @brief       受信したコストマップの情報のチェック処理
			 * @param[in]   costmap_2d::Costmap2D grid_data　地図データ
			 * @return      true:地図情報が一致　false:地図情報が不一致
			 */
			bool checkCostmapInfo(costmap_2d::Costmap2D grid_data);
	};
}
#endif

