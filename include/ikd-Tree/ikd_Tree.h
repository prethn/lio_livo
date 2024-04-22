#pragma once
#include <pcl/point_types.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <stdio.h>
#include <queue>
#include <pthread.h>
#include <chrono>
#include <time.h>

// 定义EPSS为1e-6，表示误差范围 
#define EPSS 1e-6

// 定义最小不平衡树大小为10，即当某个子树的节点数小于10时，需要进行重平衡
#define Minimal_Unbalanced_Tree_Size 10

// 定义需要重建子树的点数达到1500时，使用额外线程处理 
#define Multi_Thread_Rebuild_Point_Num 1500

// 定义DOWNSAMPLE_SWITCH为true，表示开启下采样功能 
#define DOWNSAMPLE_SWITCH true

// 定义强制重建百分比为0.2，即当某个子树中删除的节点数占总节点数的20%以上时，需要进行重建
#define ForceRebuildPercentage 0.2

// 定义支持的最大队列长度为1000000 
#define Q_LEN 1000000

using namespace std;

// TODO: 一个表示欧几里得xyz坐标、强度、法线坐标和表面曲率估计的点结构
typedef pcl::PointXYZINormal PointType;

typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector; // 定义一个使用Eigen内存分配器的点向量类型

const PointType ZeroP;

struct KD_TREE_NODE
{
    PointType point;                                             // 数据点
    int division_axis;                                            // 分割轴
    int TreeSize = 1;                                            // 总节点数
    int invalid_point_num = 0;                                   // label为删除的点的数量
    int down_del_num = 0;                                        // 下采样删除的点的数量
    bool point_deleted = false;                                      // 标记节点是否被删除
    bool tree_deleted = false;                                      // 整个tree标记为删除
    bool point_downsample_deleted = false;                         // 标记节点是否被下采样删除 
    bool tree_downsample_deleted = false;                           // 标记整个树是否被下采样删除 
    bool need_push_down_to_left = false;                             // 标记是否需要将操作向左子树下传
    bool need_push_down_to_right = false;                               // 标记是否需要将操作向右子树下传
    bool working_flag = false;                                      // 标记节点是否正在被访问
    pthread_mutex_t push_down_mutex_lock;                               // 用于多线程操作的互斥锁
    float node_range_x[2], node_range_y[2], node_range_z[2];             // tree对应的包络Box
    KD_TREE_NODE *left_son_ptr = nullptr;                           // 左子树指针对应KD_TREE
    KD_TREE_NODE *right_son_ptr = nullptr;                             // 右子树指针对应KD_TREE
    KD_TREE_NODE *father_ptr = nullptr;                              // 父树
    // For paper data record
    float alpha_del;
    float alpha_bal;
};

// 定义一个用于缓存近邻搜索结果的结构体
struct PointType_CMP{
    PointType point;// 搜索到的近邻点
    float dist = 0.0;// 搜索到的近邻点与查询点之间的距离
    PointType_CMP (PointType p = ZeroP, float d = INFINITY){ // 构造函数
        this->point = p;
        this->dist = d;
    };

    // 重载小于运算符，用于排序
    bool operator < (const PointType_CMP &a)const{
        if (fabs(dist - a.dist) < 1e-10) return point.x < a.point.x;
          else return dist < a.dist;
    }    
};

// 定义box（一个box由两个边界点定义）。
struct BoxPointType{
    float vertex_min[3]; // 定义最小顶点的坐标 
    float vertex_max[3];
};
// 枚举ikdtree中的所有操作。
enum operation_set {ADD_POINT, DELETE_POINT, DELETE_BOX, ADD_BOX, DOWNSAMPLE_DELETE, PUSH_DOWN};

// 定义一个枚举类型，用于记录删除点的存储方式
enum delete_point_storage_set {NOT_RECORD, DELETE_POINTS_REC, MULTI_THREAD_REC};

// 定义操作日志结构体 
struct Operation_Logger_Type{
    PointType point;
    BoxPointType boxpoint;
    bool tree_deleted, tree_downsample_deleted;
    operation_set op;
};

// 定义一个手动实现的队列类
class MANUAL_Q{
    private:
        int head = 0,tail = 0, counter = 0;// 定义队列的头、尾、大小
        Operation_Logger_Type q[Q_LEN];// 定义一个数组作为队列
        bool is_empty;// 定义一个标志位，表示队列是否为空
    public:
        void pop();// 弹出队列的头元素
        Operation_Logger_Type front();// 返回队列的头元素
        Operation_Logger_Type back();// 返回队列的尾元素
        void clear(); // 清空队列
        void push(Operation_Logger_Type op);// 在队列尾部插入元素
        bool empty();// 判断队列是否为空
        int size();// 返回队列大小
};

//在近邻搜索过程中，定义一个手动实现的小根堆类，用于缓存近邻搜索结果 
class MANUAL_HEAP
{
    public:
        MANUAL_HEAP(int max_capacity = 100);
        ~MANUAL_HEAP();
        void pop();// 弹出堆顶元素
        PointType_CMP top();// 返回堆顶元素
        void push(PointType_CMP point);// 向堆中插入元素
        int size();// 返回堆的大小
        void clear();// 清空堆
    private:
        PointType_CMP * heap;// 堆的数组
        void MoveDown(int heap_index);// 维护堆的性质，将新插入的元素向下冒泡
        void FloatUp(int heap_index);// 维护堆的性质，将新插入的元素向上冒泡
        int heap_size = 0;// 堆的大小
        int cap = 0;// 堆的容量
};


class KD_TREE
{
private:
    // Multi-thread Tree Rebuild
    bool termination_flag = false;  // 用于线程终止标志
    bool rebuild_flag = false;      // 用于重建标志
    pthread_t rebuild_thread;       // 重建线程
    
    // 用于线程同步的互斥锁
    // 终止标志互斥锁 -----重建指针互斥锁-----工作标志互斥锁-------搜索标志互斥锁
    pthread_mutex_t termination_flag_mutex_lock, rebuild_ptr_mutex_lock, working_flag_mutex, search_flag_mutex;
    // 重建日志互斥锁-----删除点重建互斥锁
    pthread_mutex_t rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock;
    // queue<Operation_Logger_Type> Rebuild_Logger;
    MANUAL_Q Rebuild_Logger;            // 重建日志队列 
    PointVector Rebuild_PCL_Storage;    // 重建点云存储
    KD_TREE_NODE ** Rebuild_Ptr;        // 重建根节点指针    
    int search_mutex_counter = 0;       // 用于计数查询操作的互斥锁
    static void * multi_thread_ptr(void *arg);// 重建线程函数
    void multi_thread_rebuild();        // 多线程重建函数
    void start_thread();                // 启动重建线程
    void stop_thread();                 // 停止重建线程
    void run_operation(KD_TREE_NODE ** root, Operation_Logger_Type operation);// 执行重建日志操作函数
    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0;// 用于记录树的大小和有效节点数
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0; // 平衡因子和删除因子
    float delete_criterion_param = 0.5f;// 删除节点的条件参数
    float balance_criterion_param = 0.7f;// 平衡树的条件参数
    float downsample_size = 0.2f;       // 下采样的比例
    bool Delete_Storage_Disabled = false;   // 是否禁用删除点存储
    KD_TREE_NODE * STATIC_ROOT_NODE = nullptr;  // 根节点的根，tree结构依存的基础，仅在build或彻底rebuild时改变之
    PointVector Points_deleted;         // 被删除的点的集合   
    PointVector Downsample_Storage;     // 下采样点的集合
    PointVector Multithread_Points_deleted;// 多线程删除的点的集合
    void InitTreeNode(KD_TREE_NODE * root);// 初始化树节点 
    void Test_Lock_States(KD_TREE_NODE *root);// 测试节点的锁状态
    // 构建kd-tree
    void BuildTree(KD_TREE_NODE ** root, int l, int r, PointVector & Storage);
    void Rebuild(KD_TREE_NODE ** root);// 重建kd-tree
    // 根据范围删除节点
    int Delete_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);
    // 根据点删除节点
    void Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild);
    // 根据点添加节点 
    void Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild, int father_axis);
    // 根据范围添加节点
    void Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild);
    //搜索最近点priority_queue<PointType_CMP>
    void Search(KD_TREE_NODE * root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist);//priority_queue<PointType_CMP>
    // 根据范围搜索点
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage);
    // 检查平衡因子是否满足条件
    bool Criterion_Check(KD_TREE_NODE * root);
    // 向下更新节点的信息
    void Push_Down(KD_TREE_NODE * root);
    // 更新节点的信息 
    void Update(KD_TREE_NODE * root); 
    void delete_tree_nodes(KD_TREE_NODE ** root);// 删除kd-tree的所有节点
    void downsample(KD_TREE_NODE ** root);// 下采样kd-tree
    bool same_point(PointType a, PointType b);// 判断两个点是否相同 
    float calc_dist(PointType a, PointType b);// 计算两个点之间的距离
    float calc_box_dist(KD_TREE_NODE * node, PointType point);   // 计算点到包围盒的距离  
    static bool point_cmp_x(PointType a, PointType b); // 按x轴比较两个点的大小
    static bool point_cmp_y(PointType a, PointType b); // 按y轴比较两个点的大小
    static bool point_cmp_z(PointType a, PointType b); // 按z轴比较两个点的大小
    void print_treenode(KD_TREE_NODE * root, int index, FILE *fp, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

public:
    // 构造函数，设置删除因子、平衡因子和包围盒长度
    KD_TREE(float delete_param = 0.5, float balance_param = 0.6 , float box_length = 0.2);
    ~KD_TREE();// 析构函数，销毁kd-tree
    // TODO:  设置删除因子
    void Set_delete_criterion_param(float delete_param);
    void Set_balance_criterion_param(float balance_param);// 设置平衡因子
    void set_downsample_param(float box_length);// 设置包围盒长度
    // 初始化kd-tree，并设置删除因子、平衡因子和包围盒长度 
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2); 
    int size();// 返回kd-tree的大小
    int validnum();// 返回kd-tree的有效节点数
    void root_alpha(float &alpha_bal, float &alpha_del);// 获取根节点的平衡因子和删除因子
    void Build(PointVector point_cloud);// 用点云构建kd-tree 
    // 根据点查找k个最近点
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> & Point_Distance, double max_dist = INFINITY);
    // 添加点到kd-tree，并返回添加的点数
    int Add_Points(PointVector & PointToAdd, bool downsample_on);
    // 添加点的包围盒到kd-tree
    void Add_Point_Boxes(vector<BoxPointType> & BoxPoints);
    void Delete_Points(PointVector & PointToDel);// 根据点删除节点
    int Delete_Point_Boxes(vector<BoxPointType> & BoxPoints);// 根据包围盒删除节点
    // 将kd-tree展平为点集
    void flatten(KD_TREE_NODE * root, PointVector &Storage, delete_point_storage_set storage_type);
    // 获取被删除的点的集合
    void acquire_removed_points(PointVector & removed_points);
    void print_tree(int index, FILE *fp, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    // 获取kd-tree的包围盒 
    BoxPointType tree_range();
    PointVector PCL_Storage;    // 点云存储  
    KD_TREE_NODE * Root_Node = nullptr;// 根节点指针
    int max_queue_size = 0;// 最大队列大小
};
