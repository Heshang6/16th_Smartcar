#include "image.hpp"
#include <iostream>

int f[30 * CAMERA_H];//考察连通域联通性
//每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    int   connect_num;//连通标记
}range;

//每行的所有白条子
typedef struct {
    uint8_t   num;//每行白条数量
    range   area[white_num_MAX];//该行各白条区域
}all_range;

//属于赛道的每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    uint8_t   width;//宽度
}road_range;

//每行属于赛道的每个白条子
typedef struct {
    uint8_t   white_num;
    road_range   connected[white_num_MAX];
}road;

all_range white_range[CAMERA_H];//所有白条子
road my_road[CAMERA_H];//赛道
uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];
int all_connect_num = 0;//所有白条子数
uint8_t top_road;//赛道最高处所在行数
uint32_t threshold = 160; //阈值
uint32_t prospect = 45; //前瞻
uint8_t* fullBuffer;
int img_protect = 0;


int zebra_change = 0;


////////////////////////////////////////////
//功能：二值化
//输入：灰度图片
//输出：二值化图片
//备注：
///////////////////////////////////////////
void THRE()
{
    uint8_t* map;
    uint8_t* my_map;
    map = fullBuffer;
    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if ((*map) > threshold)
                (*my_map) = 255;
            else (*my_map) = 0;
            map++;
            my_map++;
        }
    }
}
////////////////////////////////////////////
//功能：粗犷的清车头
//输入：
//输出：
//备注：要根据自己车头的大小进行修改
///////////////////////////////////////////
void head_clear(void)
{
    uint8_t* my_map;
    int count = 0;
    for (int i = 119; i >= 99; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 40; j <= 135; j++)
        {
            *(my_map + j) = white;
        }
    }
    for (int i = 100; i >= 84; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 40 + count; j <= 135 - count; j++)
        {
            *(my_map + j) = 255;
        }
        count++;
    }
}

////////////////////////////////////////////
//功能：查找父节点
//输入：节点编号
//输出：最老祖先
//备注：含路径压缩
///////////////////////////////////////////
int find_f(int node)
{
    if (f[node] == node)return node;//找到最古老祖先，return
    f[node] = find_f(f[node]);//向上寻找自己的父节点
    return f[node];
}

////////////////////////////////////////////
//功能：提取跳变沿 并对全部白条子标号
//输入：IMG[120][188]
//输出：white_range[120]
//备注：指针提速
///////////////////////////////////////////
void search_white_range()
{
    uint8_t i, j;
    int istart = NEAR_LINE;//处理起始行
    int iend = FAR_LINE;//处理终止行
    int tnum = 0;//当前行白条数
    all_connect_num = 0;//白条编号初始化
    uint8_t* map = NULL;
    for (i = istart; i >= iend; i--)
    {
        map = &IMG[i][LEFT_SIDE];//指针行走加快访问速度
        tnum = 0;
        for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
        {
            if ((*map))//遇白条左边界
            {
                tnum++;
                if (tnum >= white_num_MAX)break;
                range* now_white = &white_range[i].area[tnum];
                now_white->left = j;

                //开始向后一个一个像素点找这个白条右边界
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;
                now_white->connect_num = ++all_connect_num;//白条数加一，给这个白条编号
            }
        }
        white_range[i].num = tnum;
    }
}

////////////////////////////////////////////
//功能：寻找白条子连通性，将全部联通白条子的节点编号刷成最古老祖先的节点编号
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_all_connect()
{
    //f数组初始化
    for (int i = 1; i <= all_connect_num; i++)
        f[i] = i;

    //u为up d为down 即为当前处理的这两行中的上面那行和下面那行
    //u_num：上面行白条数
    //u_left：上面行当前白条左边界
    //u_right：上面行当前白条右边界
    //i_u：当前处理的这个白条是当前这行（上面行）白条中的第i_u个
    int u_num, i_u, u_left, u_right;
    int d_num, i_d, d_left, d_right;
    all_range* u_white = NULL;
    all_range* d_white = NULL;
    for (int i = NEAR_LINE; i > FAR_LINE; i--)//因为每两行每两行比较 所以循环到FAR_LINE+1
    {
        u_num = white_range[i - 1].num;
        d_num = white_range[i].num;
        u_white = &white_range[i - 1];
        d_white = &white_range[i];
        i_u = 1; i_d = 1;

        //循环到当前行或上面行白条子数耗尽为止
        while (i_u <= u_num && i_d <= d_num)
        {
            //变量先保存，避免下面访问写的冗杂且访问效率低
            u_left = u_white->area[i_u].left;
            u_right = u_white->area[i_u].right;
            d_left = d_white->area[i_d].left;
            d_right = d_white->area[i_d].right;

            if (u_left <= d_right && u_right >= d_left)//如果两个白条联通
                f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//父节点连起来

            //当前算法规则，手推一下你就知道为啥这样了
            if (d_right > u_right)i_u++;
            if (d_right < u_right)i_d++;
            if (d_right == u_right) { i_u++; i_d++; }
        }
    }
}

////////////////////////////////////////////
//功能：寻找赛道
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_road()
{
    uint8_t istart = NEAR_LINE;
    uint8_t iend = FAR_LINE;
    top_road = NEAR_LINE;//赛道最高处所在行数，先初始化话为最低处
    int road_f = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
    int while_range_num = 0, roud_while_range_num = 0;
    all_range* twhite_range = NULL;
    road* tmy_road = NULL;
    //寻找赛道所在连通域
    // 寻找最中心的白条子
    for (int i = 1; i <= white_range[istart].num; i++)
        if (white_range[istart].area[i].left <= CAMERA_W / 2
            && white_range[istart].area[i].right >= CAMERA_W / 2 && (white_range[istart].area[i].right - white_range[istart].area[i].left) >= 10)
            road_f = find_f(white_range[istart].area[i].connect_num);

    if (road_f == -1)//若赛道没在中间，在113行选一行最长的认为这就是赛道
    {
        int widthmax = 0, jselect = 1;
        for (int i = 1; i <= white_range[istart].num; i++)
            if (white_range[istart].area[i].right - white_range[istart].area[i].left > widthmax)
            {
                widthmax = white_range[istart].area[i].right - white_range[istart].area[i].left;
                jselect = i;
            }
        road_f = find_f(white_range[istart].area[jselect].connect_num);
    }

    //现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
    for (int i = istart; i >= iend; i--)
    {
        //变量保存，避免之后写的冗杂且低效
        twhite_range = &white_range[i];
        tmy_road = &my_road[i];
        while_range_num = twhite_range->num;
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_f(twhite_range->area[j].connect_num) == road_f)
            {
                top_road = i;
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;

            }
        }
    }
}

////////////////////////////////////////////
//功能：返回相连下一行白条子编号
//输入：i_start起始行  j_start白条标号
//输出：白条标号
//备注：认为下一行与本行赛道重叠部分对多的白条为选定赛道
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
    uint8_t j_return;
    uint8_t j;
    uint8_t width_max = 0;
    uint8_t width_new = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    uint8_t flag = 0;
    uint8_t uwidth = 0;
    uint8_t dwidth = 0;
    uint8_t d3width = 0;
    uint8_t u0width = 0;
    uint8_t dright, dleft, uright, uleft;
    uint8_t a = 0, d3right;
    uint8_t biggest = MISS;
    j_return = MISS;//如果没找到，输出255
    if (j_start > my_road[i_start].white_num)
        return MISS;
    //选一个重叠最大的
    for (j = 1; j <= my_road[i_start - 1].white_num; j++)
    {
        dleft = my_road[i_start].connected[j_start].left;
        dright = my_road[i_start].connected[j_start].right;
        uleft = my_road[i_start - 1].connected[j].left;
        uright = my_road[i_start - 1].connected[j].right;
        uwidth = my_road[i_start - 1].connected[j].width;
        u0width = my_road[i_start - 1].connected[j + 1].width;
        dwidth = my_road[i_start].connected[j_start].width;
        ////////////////////////////////////////////////////////////////////////////////////////
        if (//宮젯
            dleft < uright
            &&
            dright > uleft
            )
        {
            //셕炬路딸댕鬼
            if (dleft < uleft) left = uleft;
            else left = dleft;

            if (dright > uright) right = uright;
            else right = dright;

            width_new = right - left + 1;

            if (width_new > width_max)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    return j_return;
}

////////////////////////////////////////////
//功能：通用决定双边
//输入：
//输出：
//备注：
///////////////////////////////////////////
void ordinary_two_line(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t j_continue[CAMERA_H];//第一条连通路径
    uint8_t i_start;
    uint8_t i_end;
    uint8_t j_start = MISS;
    int width_max;
    //寻找起始行最宽的白条子
    i_start = NEAR_LINE;
    i_end = FAR_LINE;
    width_max = 0;
    for (j = 1; j <= my_road[i_start].white_num; j++)
    {
        if (my_road[i_start].connected[j].width > width_max)
        {
            width_max = my_road[i_start].connected[j].width;
            j_start = j;
        }
    }
    j_continue[i_start] = j_start;

    //记录连贯区域编号
    for (i = i_start; i >= i_end; i--)
    {
        //如果相连编号大于该行白条数，非正常，从此之后都MISS
        if (j_continue[i] > my_road[i].white_num)
            j_continue[i - 1] = MISS;
        else
        {
            j_continue[i - 1] = find_continue(i, j_continue[i]);
        }

    }
    //全部初始化为MISS
    my_memset(left_line, MISS, CAMERA_H);
    my_memset(right_line, MISS, CAMERA_H);
    for (i = i_start; i >= i_end; i--)
    {
        if (j_continue[i] <= my_road[i].white_num)
        {
            left_line[i] = my_road[i].connected[j_continue[i]].left;
            right_line[i] = my_road[i].connected[j_continue[i]].right;
            //std::cout << unsigned(left_line[i]) << std::endl;
            //std::cout << unsigned(my_road[i].connected[j_continue[i]].width) << std::endl;
            //std::cout << unsigned(i) << std::endl;
            //IMG[i][left_line[i]] = blue;
            IMG[i][right_line[i]] = green;
        }
        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
}

int32_t check_leftline(uint8_t a, uint8_t i, int32_t up_down, uint8_t diff)
{
    int32_t point, point1;
    uint8_t flag = 0;
    uint8_t time = 0;
    uint8_t step = a;
    for (time; time < i; time++, step--)
    {
        point = left_line[step] - left_line[step + up_down];
        if (up_down < 0)
        {
            if (point <= 0 && point >= -3 && (unsigned(left_line[step]) != 0))
            {
                flag++;
            }
        }
        else if (up_down > 0)
        {
            if (point <= 2 && point >= 0 && (unsigned(left_line[step]) != 0))
            {
                flag++;
            }
        }
    }
    if (flag + diff >= i)
        return 1;
    else
        return 0;
}
int check_rightline(uint8_t a, uint8_t i, int32_t up_down, uint8_t diff)
{
    int32_t point, point1;
    uint8_t flag = 0;
    uint8_t time = 0;
    uint8_t step = a;
    for (time; time < i; time++, step--)
    {
        point = right_line[step + up_down] - right_line[step];
        if (up_down < 0)
        {
            if (point <= 3 && point >= -1 && (unsigned(right_line[step]) != 187) && (unsigned(right_line[step]) != 0)) {
                flag++;
            }
        }
        else if (up_down > 0)
        {
            if (point <= 2 && point >= 0 && (unsigned(right_line[step]) != 255) && (unsigned(right_line[step]) != 187) && (unsigned(right_line[step]) != 0)) {
                flag++;
            }
        }
    }
    if (flag + diff >= i)
        return 1;
    else
        return 0;
}
int check_midline(uint8_t a, uint8_t i, int32_t up_down)
{
    int32_t point, point1;
    uint8_t flag = 0;
    uint8_t time = 0;
    uint8_t step = a;
    for (time; time < i; time++, step--)
    {
        point = mid_line[step] - mid_line[step + up_down];
        if (point < 1 && point > -1 && (unsigned(mid_line[step]) != 255) && (unsigned(mid_line[step]) != 187) && (unsigned(mid_line[step]) != 0))
            flag++;
    }
    if (flag >= i)
        return 1;//识别出为直道
    else
        return 0;
}
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
void get_mid_line(void);
uint8_t scan_pic()
{
    uint8_t i, k, cheak = 0, l_cheak = 0, r_cheak = 0;
    int32_t num = 0, num1 = 0, left_or_right = 0;
    i = check_leftline(113, 100, -1, 3);
    k = check_rightline(113, 100, -1, 3);
    //std::cout << unsigned(i) << std::endl;
    //std::cout << unsigned(k) << std::endl;
    if (i == 1 && k == 1)
    {
        int test = 0;
        int flag = 0;
        int change = 0;
        int count = 0;
        for (int n = 25; n <= 100; n++)
        {
            change = 0;
            test = 0;
            flag = 0;
            for (int m = 45; m <= 125; m++)
            {
                if (IMG[n][m] == 0)
                    test = 0;
                else
                    test = 1;
                if (test != flag)
                {
                    change++;//发生出现突变时，change加一，同时保存变化，以便下次出现变化可识别
                    flag = test;
                }
            }
            if (change >= 16)//共八条斑马线，进出各一次，共十六次，加上边界18次，允许部分误差
                count++;
        }
        if (count > 1)
            return 5;//返回斑马线
        else
            return 0;//返回直道
    }
    /*if (i == 1 && k == 1)
    {
        int flag = 0;
        int test = 0;
        int change = 0;
        for (int m = 0; m <= 187; m++)
        {
            if (IMG[60][m] == 0)
                test = 0;
            else
                test = 1;
            if (test != flag)
            {
                change++;//发生出现突变时，change加一，同时保存变化，以便下次出现变化可识别
                flag = test;
            }
        }
        if (change >= 16)//共八条斑马线，进出各一次，共十六次，加上边界18次，允许部分误差
            return 5;//返回斑马线
        else
            return 0;//返回直道
    }*/
    else
    {
        for (uint8_t a = 113; a > 1; a--) {
            int32_t point = right_line[a] - left_line[a];
            if (point > 175)
                cheak++;//当左右边界差距过大时，cheak加一
        }
        if (cheak > 2)
            return 1;//识别出十字
        else {
            uint8_t temp = 0;
            for (uint8_t a = 113; a > 1; a--) {
                int32_t point = left_line[a - 1] - left_line[a];
                if (point < -3) {
                    temp = a;
                    break;
                }
            }
            for (uint8_t a = temp; a > 1; a--) {
                int32_t point = left_line[a - 1] - left_line[temp];
                if (point < -10)
                    l_cheak++;
            }
            temp = 0;
            for (uint8_t a = 113; a > 1; a--) {
                int32_t point = right_line[a - 1] - right_line[a];
                if (point < -3) {
                    temp = a;
                    break;
                }
            }
            for (uint8_t a = temp; a > 1; a--) {
                int32_t point = right_line[a - 1] - right_line[temp];
                if (point < -10)
                    r_cheak++;
            }
            if (l_cheak > 4 && r_cheak > 4)
                return 2;
            else
            {
                get_mid_line();
                for (uint8_t i = 100; i >= FAR_LINE; i--)
                {
                    num = mid_line[i] - mid_line[i - 1];
                    num1 = mid_line[i - 1] - mid_line[i - 2];
                    if (num == 1 && num1 == 1)
                        left_or_right++;
                    else if (num == -1 && num1 == -1)
                        left_or_right--;
                    else if (num > 10 || num < -10)
                        break;
                }
                if (left_or_right > 0)
                    return 3;
                else if (left_or_right < 0)
                    return 4;
            }
        }
    }

}
void run_on_ten_withoout_bottom()
{
    uint8_t i, cheak = 0;
    int32_t flag = 0;
    for (uint8_t a = 1; a < 113; a++) {
        int32_t point = right_line[a] - left_line[a];
        if (point > 175)
        {
            cheak++;
            flag = a;
            break;
        }
    }
    flag = flag - 6;
    for (flag; flag > 1; flag--) {
        i = check_midline(flag, 7, -1);
        std::cout << unsigned(i) << std::endl;
        if (i == 1)
        {
            std::cout << flag << std::endl;
            uint8_t a = flag - 1;
            for (a; a < 113; a++)
            {
                //std::cout << unsigned(a) << std::endl;
                mid_line[a + 2] = mid_line[flag];
            }
            break;
        }
    }
}
void run_on_ten_with_bottom()
{
    uint8_t i, cheak = 0, l_cheak = 0, r_cheak = 0;
    uint8_t left_gap, right_gap;
    int32_t res_left_up = 0, res_left_down = 0;
    int32_t res_right_up = 0, res_right_down = 0;
    int32_t gap;
    float slope;
    for (uint8_t i = 113; i > 1; i--) {
        if (check_leftline(i, 1, -1, 0) != 1) {
            res_left_up = i;
            break;
        }
    }
    for (uint8_t i = 10; i < 113; i++) {
        if (check_leftline(i, 1, +1, 0) != 1) {
            res_left_down = i;
            break;
        }
    }
    for (uint8_t i = 113; i > 1; i--) {
        if (check_rightline(i, 1, -1, 0) != 1) {
            res_right_up = i;
            break;
        }
    }
    for (uint8_t i = 10; i < 113; i++) {
        if (check_rightline(i, 1, +1, 0) != 1) {
            res_right_down = i;
            break;
        }
    }
    //std::cout << unsigned(res_left_up) << std::endl;
    //std::cout << unsigned(res_left_down) << std::endl;
    //std::cout << unsigned(res_right_up) << std::endl;
    //std::cout << unsigned(res_right_down) << std::endl;
    left_gap = res_left_up - res_left_down;
    right_gap = res_right_up - res_right_down;
    gap = res_right_up - res_left_up;
    l_cheak = check_leftline(113, 31, -1, 0);
    //std::cout << unsigned(cheak) << std::endl;
    //std::cout << unsigned(res_right_down) << std::endl;
    //std::cout << gap << std::endl;
    if (l_cheak == 1 && gap < 10 && gap > -10)
        //gap <= 10 && gap >= -10 && res_left_up > 20 && res_left_down < 30)
    {
        //std::cout << res_left_down << std::endl;
        //std::cout << res_left_up << std::endl;
        for (i = 0; i <= left_gap + 4; i++)
        {
            slope = (float)(left_gap + 8) / (float)(left_line[res_left_down - 4] - left_line[res_left_up + 4]);
            left_line[res_left_up + 2 - i] = (float)i / slope + left_line[res_left_up];
            IMG[res_left_up + 2 - i][left_line[res_left_up + 2 - i]] = blue;

        }
        for (i = 0; i <= right_gap + 4; i++)
        {
            slope = (float)(right_gap + 8) / (float)(right_line[res_right_down - 4] - right_line[res_right_up + 4]);
            right_line[res_right_up + 2 - i] = (float)i / slope + right_line[res_right_up];
            IMG[res_right_up + 2 - i][right_line[res_right_up + 2 - i]] = green;
        }
    }
}
void run_left()
{
    int32_t num = 0, num1 = 0, num2 = 0, point = 0, gap = 0;
    for (uint8_t i = 100; i >= 40; i--)
    {
        num = mid_line[i] - mid_line[i - 1];
        num1 = mid_line[i - 1] - mid_line[i - 2];
        num2 = mid_line[i - 2] - mid_line[i - 3];
        if (num == 1 && num1 == 1)
            point = i;
        else if (num > 3 || num < -3)
            break;
    }
    if (point != 0)
    {
        std::cout << point << std::endl;
        gap = right_line[point] - mid_line[point];
        for (uint8_t i = point; i >= FAR_LINE; i--)
        {
            if ((int32_t)right_line[i] - gap > 0)
                mid_line[i] = right_line[i] - gap;
            else
                mid_line[i] = MISS;
        }
    }
}
void run_right()
{
    int32_t num = 0, num1 = 0, num2 = 0, point = 0, gap = 0;
    for (uint8_t i = 100; i >= FAR_LINE; i--)
    {
        num = mid_line[i] - mid_line[i - 1];
        num1 = mid_line[i - 1] - mid_line[i - 2];
        num2 = mid_line[i - 2] - mid_line[i - 3];
        if (num == -1 && num1 == -1)
            point = i;
        else if (num > 3 || num < -3)
            break;

    }
    if (point != 0)
    {
        gap = left_line[point] - mid_line[point];
        for (uint8_t i = point; i >= FAR_LINE; i--)
        {
            if ((int32_t)left_line[i] - gap > 0)
                mid_line[i] = left_line[i] - gap;
            else
                mid_line[i] = MISS;
        }
    }
}



int check_zebra(void)
{
    int test = 0;
    int flag = 0;
    int change = 0;
    int count = 0;
    for (int m = 40; m >= 25; m--)
    {
         change = 0;
         test = 0;
         flag = 0;
                for (int n = 45; n <= 125; n++)
                {
                    if (IMG[m][n] == 0)
                        test = 0;
                    else
                        test = 1;
                    if (test != flag)
                    {
                        change++;//发生出现突变时，change加一，同时保存变化，以便下次出现变化可识别
                        flag = test;
                    }
                }
                if (change >= 14)//共八条斑马线，进出各一次，共十六次，加上边界18次，允许部分误差
                {
                    count++;
                }
    }
    if (count>=3)
    {
        return 1;
    }
    else
        return 0;
}
////////////////////////////////////////////
//功能：数组初始化
//输入：uint8_t* ptr 数组首地址, uint8_t num初始化的值, uint8_t size数组大小
//输出：
//备注：因为k66库中认为memset函数不安全，所以无法使用；因此需要自己写一个my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
    uint8_t* p = ptr;
    uint8_t my_num = num;
    uint8_t Size = size;
    for (int i = 0; i < Size; i++, p++)
    {
        *p = my_num;
    }
}
////////////////////////////////////////////
//功能：中线合成
//输入：左右边界
//输出：中线
//备注：
///////////////////////////////////////////
void get_mid_line(void)
{
    my_memset(mid_line, MISS, CAMERA_H);
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
    {
        if (left_line[i] != MISS)
        {
            mid_line[i] = (left_line[i] + right_line[i]) / 2;
        }
        else
        {
            mid_line[i] = MISS;
        }
    }
    int i = 112;
    while (i > FAR_LINE)
    {
        if (mid_line[i] == mid_line[i - 2])
            mid_line[i - 1] = mid_line[i];
        else if (mid_line[i] == mid_line[i - 2] + 1)
            mid_line[i - 1] = mid_line[i];
        else if (mid_line[i] == mid_line[i - 2] - 1)
            mid_line[i - 1] = mid_line[i];
        i--;
    }
}

void scan_mid_line(void)
{
    int i, j, m, n;
    int left[120];
    int right[120];
    for (i = 0; i < 119; i++)
    {
        left[i] = 90;
        right[i] = 90;
    }
    for (i = 15; i < 119; i++)
    {
        for (m = 90; m > 0; m--)
        {
            if ((IMG[i][m] == 0) || (IMG[i][m] == MISS))
            {
                left[i] = m;
                break;
            }
        }
        for (n = 91; n < 187; n++)
        {
            if ((IMG[i][n] == 0) || (IMG[i][n] == MISS))
            {
                right[i] = n;
                break;
            }
        }
        mid_line[i] = (left[i] + right[i])/2;
    }
}

////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
void image_main()
{
    uint8_t flag;
    int coun = 0;
    THRE();
    for(int i = 89; i<119;i++)
    {
        for (int j=0;j<187;j++)
        {
            if (IMG[i][j]==0)
            {
                coun++;
            }
        }
    }
    if (coun>5000)
        img_protect = 1;
    if(coun <5000)
        img_protect = 0;
    head_clear();
    zebra_change=check_zebra();
    search_white_range();
    find_all_connect();
    find_road();
    /*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/
    ordinary_two_line();
    flag = scan_pic();
    if (flag == 0) {
        //std::cout << unsigned(flag) << std::endl;
        get_mid_line();
    }
    else if (flag == 1) {
        get_mid_line();
        run_on_ten_withoout_bottom();
    }
    else if (flag == 2) {
        run_on_ten_with_bottom();
        get_mid_line();
        //std::cout << unsigned(flag) << std::endl;
    }
    else if (flag == 3) {
        get_mid_line();
        run_left();
        //std::cout << unsigned(flag) << std::endl;
    }
    else if (flag == 4)
    {
        //std::cout << unsigned(flag) << std::endl;
        get_mid_line();
        run_right();
    }
    else if (flag == 5)
    {
        scan_mid_line();
    }
    //scan_breakpoint2();
    if (mid_line[119] == MISS)
            mid_line[119] = 89;
        for (int i = 118; i <= 0; i--)
        {
            if (mid_line[i] == MISS)
            {
                mid_line[i] = mid_line[i + 1];
            }
        }
    /*  */
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
            IMG[i][mid_line[i]] = 0;
    for (int a = 0; a <= 119; a++)
        IMG[a][94] = 0;
}
