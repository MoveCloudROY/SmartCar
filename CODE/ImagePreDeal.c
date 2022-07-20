/*
 * @Author: ROY1994
 * @Date: 2022-02-04 13:40:22
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:40:52
 * @FilePath: \myImageDeal_v0.1\ImagePreDeal.cpp
 * @Description: 存放预处理函数
 */

#include "ImagePreDeal.h"

extern ConstDataTypeDef ConstData;
extern uint8_t mt9v03x_image[120][188];
uint8_t imageCut[HEIGHT][WIDTH];
uint8_t imageBin[HEIGHT][WIDTH];


/**
 * @description: 预处理调用函数，目前支持OTSU大津法、SAUVOLA局部二值化、中值滤波
 *                  调用顺序： OTSU / 中值滤波 -> SAUVOLA
 * @param {PreDealMethodEnum} method
 * @return {*}
 */
void img_preProcess(PreDealMethodEnum method)
{
    uint8_t th_otsu;
    switch (method)
    {
        case OTSU:
        {
            th_otsu = otsu();
#if defined (DEBUG)
            printf("@@ th_otsu: %d @@\n", th_otsu);
#endif
            int thre_tmp = max(th_otsu, ConstData.kImageOtsuStaticTh), thre;
            thre = thre_tmp;
            for (int i = 0; i < HEIGHT; ++i)
            {
                for (int j = 0; j < WIDTH; ++j)
                {
//                    if (j < 15)
//                        thre = thre_tmp - 10;
//                    else if (j >= WIDTH - 15)
//                        thre = thre_tmp - 10;
//                    else
//                        thre = thre_tmp;
                    imageBin[i][j] = (mt9v03x_image[i][j] > thre) ? 255 : 0;
                }
            }
            // *(imageBin[0] + i) = *(mt9v03x_image[0] + i) > th_otsu?255:0;
            break;
        }
        case OTSU2D:
            //二维OTSU，占个位，不知道用的到吗
            break;
        case SAUVOLA:
            // uint8_t th_otsu = otsu();
            // for (int i = 0; i < HEIGHT * WIDTH; i++)
            //     *(imageBin[0] + i) = *(mt9v03x_image[0] + i) > th_otsu?255:0;
            sauvola();
            break;
        case SOBEL:
            sobel();
            break;
        case GAUSSIAN_FILTER:
            break;
        case MEDIAN_FILTER:
            median_filter();
            break;
        case MORPH_EROSION:
            morph_erosion();
            break;
        case MORPH_DILITION:
            morph_dilition();
            break;
        case MY_MORPH_OPEN:
            morph_open();
            break;
        case MY_MORPH_CLOSE:
            morph_close();
            break;

        default:
            break;
    }
}

void compress(void)
{
    for(int i = 0; i < HEIGHT; i++)
    {
        for(int j = 0; j < WIDTH; j++)
        {
            if(i < HEIGHT/2)
            {
                *(imageBin[i] + j) = *(mt9v03x_image[i] + j);
            }
            else
            {
                *(imageBin[i] + j) = 0;
            }
        }
    }
}

/**
 * @description: 大津法（全局二值化），原图像 -> 处理后图像
 * @param {*}
 * @return {uint8_t} th 二值化阈值
 */
uint8_t otsu(void)
{
    int histogram[256] = {0};
    int i_mult_histogram[256] = {0};
    int total = HEIGHT * WIDTH;
    int u0_sum = 0,u1_sum = 0,sum_i_mult_h = 0,n0 = 0,n1 = 0;
    float max_value = 0.0,n_value = 0.0,u0 = 0.0,u1 = 0.0;
    uint8_t th = 0,minn = 255,maxn = 0;

    uint8_t * p_pixel = mt9v03x_image[0];
    
    for(int i=0; i<total; ++i)//计算灰度直方图
    {
        histogram[p_pixel[i]]++;
        if(minn>p_pixel[i]) minn = p_pixel[i];
        if(maxn<p_pixel[i]) maxn = p_pixel[i];

    }

    for(int i=minn; i<=min(ConstData.kImageOtsuBrightLimit, maxn); ++i)//获取灰度权值和（使用16位避免爆
    {
        i_mult_histogram[i] = i*histogram[i];
        sum_i_mult_h += i_mult_histogram[i];//求总和,方便之后处理
    }

    for (int i=minn; i<min(ConstData.kImageOtsuBrightLimit, maxn); ++i)//从最小开始遍历，寻找最合适的阈值
    {
        //计算前景、后景像素点个数
        n0 += histogram[i];
        n1 = total - n0;
        
        //计算前景、后景灰度值和
        u0_sum += i_mult_histogram[i];
        u1_sum = sum_i_mult_h - u0_sum;

        u0 = u0_sum / (float)n0;
        u1 = u1_sum / (float)n1;
        
        n_value = (u0 - u1) * (u0 - u1) * (float)n0 * n1 ;//float在前，保证显式转化

        if (n_value>max_value)
        {
            max_value = n_value;
            th = i;
        }
    }
    return th;
}

/**
 * @description: sauvola局部二值化（光照方案一），原图像 -> 处理后图像
 * @param {*}
 * @return {*}
 */
void sauvola(void)
{

    int i, j, left, right, down, up, sum, sqsum;
    float mean, v, tmp, area;
    int *integralCols = (int *)malloc(WIDTH * sizeof(int));
    int *integralColsSqu = (int *)malloc(WIDTH * sizeof(int));
    for (int j = 0; j < WIDTH; ++j) //对每一列预处理
    {
        integralCols[j] = 0;
        integralColsSqu[j] = 0;
        for (int i = 0; i < kernel_sizeby2_sauvola; ++i) // kernel center 和 后 kernel_sizeby2_sauvola - 1行 之和
        {
            integralCols[j] += mt9v03x_image[i][j];
            integralColsSqu[j] += mt9v03x_image[i][j] * mt9v03x_image[i][j];
        }
    }
    // memset(integralCols, 0, sizeof(integralCols));
    // memset(integralColsSqu, 0, sizeof(integralColsSqu));
    for (i = 0; i < HEIGHT; ++i)
    {
        up = i - kernel_sizeby2_sauvola;
        down = i + kernel_sizeby2_sauvola;
        for (j = 0; j < WIDTH; ++j) //纵向更新每一列的窗口,分三种情况讨论，起始，中途，结尾
        {

            if (up <= 0) //(i <= kernel_sizeby2_sauvola)//起始部分加入下端
            {
                // downi = down * WIDTH + j;
                integralCols[j] += mt9v03x_image[down][j];
                integralColsSqu[j] += mt9v03x_image[down][j] * mt9v03x_image[down][j];
            }
            else if (down >= HEIGHT) //(h - 1 - i < kernel_sizeby2_sauvola) //结尾部分减去上端
            {
                // upi = (up - 1) * WIDTH + j;
                integralCols[j] -= mt9v03x_image[up - 1][j];
                integralColsSqu[j] -= mt9v03x_image[up - 1][j] * mt9v03x_image[up - 1][j];
            }
            else //中间部分减去上端加入下端
            {
                // upi = (up - 1) * WIDTH + j;
                // downi = down * WIDTH + j;
                integralCols[j] = integralCols[j] + mt9v03x_image[down][j] - mt9v03x_image[up - 1][j];
                integralColsSqu[j] = integralColsSqu[j] + mt9v03x_image[down][j] * mt9v03x_image[down][j] - mt9v03x_image[up - 1][j] * mt9v03x_image[up - 1][j];
                // printf("%d\n",mt9v03x_image[up - 1][j]);
            }
            // if(integralCols[j]<0) printf("%d\n",i);
            // printf("%d\n",integralCols[j]);
        }
        sum = 0;
        sqsum = 0;
        for (j = 0; j < kernel_sizeby2_sauvola; ++j) //横向预处理
        {
            sum += integralCols[j];
            sqsum += integralColsSqu[j];
        }
        for (j = 0; j < WIDTH; ++j) //横向滑窗
        {
            left = j - kernel_sizeby2_sauvola;
            right = j + kernel_sizeby2_sauvola;
            area = (min(HEIGHT - 1, down) - max(0, up) + 1) * (min(WIDTH - 1, right) - max(0, left) + 1); //面积
            if (left <= 0)                                                                       //横向更新最左侧
            {
                sum += integralCols[right];
                sqsum += integralColsSqu[right];
            }
            else if (right >= WIDTH) //横向更新最右侧
            {
                sum -= integralCols[left - 1];
                sqsum -= integralColsSqu[left - 1];
            }
            else //中间部分
            {
                sum = sum + integralCols[right] - integralCols[left - 1];
                sqsum = sqsum + integralColsSqu[right] - integralColsSqu[left - 1];
            }
            //算参数，怎么来的可以看论文里我的标注
            mean = sum / area;
            v = sqsum / area - mean * mean;

            tmp = mt9v03x_image[i][j] + mean * (k_sauvola - 1.0);
            if (tmp <= 0 || tmp * tmp <= k2_sauvola * mean * mean * v / r2_sauvola)
                imageBin[i][j] = 0; //实际用的时候直接用01好了
            else
                imageBin[i][j] = 255;
        }
    }

    free(integralCols);
    free(integralColsSqu);
}


/**
 * @description: sobel边缘提取（光照方案三），原图像 -> 处理后图像
 * @param {*}
 * @return {*}
 */
void sobel(void)
{
    uint16_t i, j;
    //uint8_t temp[HEIGHT][WIDTH] = {0};
    for (i = 1; i <= HEIGHT - 2; ++i)
    {
        for (j = 1; j <= WIDTH - 2; ++j)
        {
            int16_t sumx=0,sumy=0;
            sumy = -1 * mt9v03x_image[i - 1][j - 1] - 2 * mt9v03x_image[i - 1][j] - 1 * mt9v03x_image[i - 1][j + 1]
                + 1 * mt9v03x_image[i + 1][j - 1] + 2 * mt9v03x_image[i + 1][j] + 1 * mt9v03x_image[i + 1][j + 1];
            
            sumx = -1 * mt9v03x_image[i - 1][j - 1] - 2 * mt9v03x_image[i][j - 1] - 1 * mt9v03x_image[i + 1][j - 1]
                + 1 * mt9v03x_image[i - 1][j + 1] + 2 * mt9v03x_image[i][j + 1] + 1 * mt9v03x_image[i + 1][j + 1];
            if(abs(sumx)+abs(sumy) > TH_SOBEL)
                imageBin[i][j] = 0;
            else 
                imageBin[i][j] = 255;
        }

    }
}

/**
 * @description: 中值滤波，原图像 -> 原图像
 * @param {*}
 * @return {*}
 */
void median_filter(void)
{
    int i = -1, j = 0, k, f = -1, cnt = 0, sum = 0, ret = 0, total = HEIGHT * WIDTH;
    int up, down, left, right, tmp, addtmp;
    unsigned int tr[257] = {0};
    uint8_t temp[HEIGHT][WIDTH];
    for(int i = 0; i < kernel_sizeby2_medianfilter ; ++i)
    {
        for(int j = 0; j < kernel_sizeby2_medianfilter + 1; ++j)
        {
            tmp = mt9v03x_image[i][j] + 1;
            while(tmp <= N)
            {
                tr[tmp] += 1;
                tmp += lowbit(tmp);
            }
        }
    }
    tmp = 1;//加上0的数量
    addtmp = kernel_2_size_medianfilter - kernel_sizeby2_medianfilter*(kernel_sizeby2_medianfilter + 1);
    while(tmp <= N)
    {
        tr[tmp] += addtmp;
        tmp += lowbit(tmp);
    }
    while(cnt!=total)
    {
        ++cnt;//计数

        //更新滑动窗口
        if(cnt % WIDTH == 1)//左右滑动遇到边后向下滑动
        {
            ++i;
            f = -f;

            up = i - kernel_sizeby2_medianfilter;
            down = i + kernel_sizeby2_medianfilter;
            left = max(j - kernel_sizeby2_medianfilter, 0);
            right = min(j + kernel_sizeby2_medianfilter, WIDTH - 1);
            if(up <= 0)//上侧顶到边则加入下册，上侧减0的个数
            {
                for(k = left; k <= right; ++k)
                {
                    tmp = mt9v03x_image[down][k] + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] += 1;
                        tmp += lowbit(tmp);
                    }
                }
                tmp = 1;//减去0的数量
                addtmp = right - left + 1;
                while(tmp <= N)
                {
                    tr[tmp] -= addtmp;
                    tmp += lowbit(tmp);
                }
            }
            else if(down >= HEIGHT)
            {
                for(k = left; k <= right; ++k)
                {
                    tmp = mt9v03x_image[up - 1][k] + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] -= 1;
                        tmp += lowbit(tmp);
                    }
                }
                tmp = 1;//加上0的数量
                addtmp = right - left + 1;
                while(tmp <= N)
                {
                    tr[tmp] += addtmp;
                    tmp += lowbit(tmp);
                }
            }
            else
            {
                for(k = left; k <= right; ++k)
                {
                    tmp = mt9v03x_image[down][k] + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] += 1;
                        tmp += lowbit(tmp);
                    }

                    tmp = mt9v03x_image[up - 1][k] + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] -= 1;
                        tmp += lowbit(tmp);
                    }
                }
                //中间滑动不影响0的个数
            }
        }
        else//其余情况按f值左右滑动，f=-1左滑，f=1右滑
        {

            if(f == 1)//向右滑动
            {
                ++j;
                up = max(i - kernel_sizeby2_medianfilter, 0);
                down = min(i + kernel_sizeby2_medianfilter, HEIGHT - 1);
                left = j - kernel_sizeby2_medianfilter;
                right = j + kernel_sizeby2_medianfilter;
                if(left <= 0)//左侧顶到边则加入右侧，左侧减0的个数
                {
                    for(k = up; k <= down; ++k)
                    {
                        tmp = mt9v03x_image[k][right] + 1;
                        while(tmp <= N)
                        {
                            tr[tmp] += 1;
                            tmp += lowbit(tmp);
                        }
                    }
                    tmp = 1;//减去0的数量
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] -= addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else if(right >= WIDTH)
                {
                    for(k = up; k <= down; ++k)
                    {
                        tmp = mt9v03x_image[k][left - 1] + 1;
                        while(tmp <= N)
                        {
                            tr[tmp] -= 1;
                            tmp += lowbit(tmp);
                        }
                    }
                    tmp = 1;//加上0的数量
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] += addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else//中间滑动不影响0的个数
                {
                    for(k = up; k <= down; ++k)
                    {
                        tmp = mt9v03x_image[k][right] + 1;
                        while(tmp <= N)
                        {
                            tr[tmp] += 1;
                            tmp += lowbit(tmp);
                        }

                        tmp = mt9v03x_image[k][left - 1] + 1;
                        while(tmp <= N)
                        {
                            tr[tmp] -= 1;
                            tmp += lowbit(tmp);
                        }
                    }
                }
            }
            else
            {
                --j;
                up = max(i - kernel_sizeby2_medianfilter, 0);
                down = min(i + kernel_sizeby2_medianfilter, HEIGHT - 1);
                left = j - kernel_sizeby2_medianfilter;
                right = j + kernel_sizeby2_medianfilter;
                if(left < 0)//左侧顶到边则减去右侧，左侧加0的个数
                {
                    for(k = up; k <= down; ++k)
                    {
                        tmp = mt9v03x_image[k][right + 1] + 1;
                        while(tmp <= N)
                        {
                            tr[tmp] -= 1;
                            tmp += lowbit(tmp);
                        }
                    }
                    tmp = 1;//加上0的数量
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] += addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else if(right >= WIDTH - 1)//右侧顶到加入左侧，减去右侧0的个数
                {
                    for(k = up; k <= down; ++k)
                    {
                        tmp = mt9v03x_image[k][left] + 1;
                        while(tmp <= N)
                        {
                            tr[tmp] += 1;
                            tmp += lowbit(tmp);
                        }
                    }
                    tmp = 1;//加上0的数量
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] -= addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else//中间滑动不影响0的个数
                {
                    for(k = up; k <= down; ++k)
                    {
                        tmp = mt9v03x_image[k][left] + 1;
                        while(tmp <= N)
                        {
                            tr[tmp] += 1;
                            tmp += lowbit(tmp);
                        }

                        tmp = mt9v03x_image[k][right + 1] + 1;
                        while(tmp <= N)
                        {
                            tr[tmp] -= 1;
                            tmp += lowbit(tmp);
                        }
                    }

                }
            }
        }
        //更新结束
        //获取中位数赋值

        sum = 0; ret = 0;
        for(int t = 8; ~t; --t)
        {
            ret += 1 << t;                      // 尝试扩展
            if (ret >= N || sum + tr[ret] >= kernel_mid_medianfilter)  // 如果扩展失败
                ret -= 1 << t;
            else
                sum += tr[ret];  // 扩展成功后 要更新之前求和的值
        }
        temp[i][j] = ret;
        //printf("%d %d %d %u\n", i, j, ret, tr[1]);
    }
    memcpy(mt9v03x_image,temp,sizeof(temp));
}


/**
 * @description: 腐蚀
 * @param {*}
 * @return {*}
 * 太慢了, 考虑具体问题,我们只需要滤除赛道上的黑点,故周围只要白点数量多于阈值即可
 */
void morph_erosion(void)
{
    for (int i = 0; i < HEIGHT ; i++)
    {
        for (int j = 0; j < WIDTH ; j++)
        {
            if((i - 1 >= 0  && !!imageBin[i-1][j]) +  (i + 1 < HEIGHT && !!imageBin[i+1][j]) +
                    (j - 1 >= 0 && !!imageBin[i][j-1]) +  (j + 1 < WIDTH && !!imageBin[i][j+1]) >= 3)
                imageBin[i][j] = 255;
            
        }
    }
}

/**
 * @description: 膨胀
 * @param {*}
 * @return {*}
 */
void morph_dilition(void)
{
    if (WIDTH - kernel_size_morph < 0 || HEIGHT - kernel_size_morph < 0) return;

    //int kernel_sizeby2_morph = kernel_sizeby2_morph + 1;
    uint8_t temp[HEIGHT][WIDTH];
    for(int i = 0; i < kernel_sizeby2_morph; ++i)
    {
        for(int j = 0; j < WIDTH; ++j)
        {
            temp[i][j] = imageBin[i][j];
            temp[HEIGHT - 1 - i][j] = imageBin[HEIGHT - 1 - i][j];
        }
    }
    for(int i = 0; i < HEIGHT; ++i)
    {
        for(int j = 0; j < kernel_sizeby2_morph; ++j)
        {
            temp[i][j] = imageBin[i][j];
            temp[i][WIDTH - 1 - j] = imageBin[i][WIDTH - 1 - j];
        }
    }
    int val = 1;
    for (int i = kernel_sizeby2_morph; i < HEIGHT - kernel_sizeby2_morph; i++)
    {
        for (int j = kernel_sizeby2_morph; j < WIDTH - kernel_sizeby2_morph; j++)
        {
            for (int m = -kernel_sizeby2_morph; m <= kernel_sizeby2_morph; m++)
            {
                for (int n = -kernel_sizeby2_morph; n <= kernel_sizeby2_morph; n++)
                {
                    // if (m == kernel_sizeby2_morph && n == kernel_sizeby2_morph )
                    //      continue;
                        val &= !!imageBin[i + m][j + n];
                }
            }

            temp[i][j] = val*255;
            val = 1;
        }
    }

    memcpy(imageBin,temp,sizeof(temp));
}

/**
 * @description: 开运算，先腐蚀后膨胀
 * @param {*}
 * @return {*}
 */
void morph_open(void)
{
    morph_erosion();
    morph_dilition();
}

/**
 * @description: 闭运算，先膨胀后腐蚀
 * @param {*}
 * @return {*}
 */
void morph_close(void)
{
    morph_dilition();
    morph_erosion();
}
