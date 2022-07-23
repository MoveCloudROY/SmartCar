/*
 * @Author: ROY1994
 * @Date: 2022-02-04 13:40:22
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:40:52
 * @FilePath: \myImageDeal_v0.1\ImagePreDeal.cpp
 * @Description: 锟斤拷锟皆わ拷锟斤拷锟斤拷锟斤拷锟�
 */

#include "ImagePreDeal.h"

extern ConstDataTypeDef ConstData;
extern uint8_t mt9v03x_image[120][188];
uint8_t imageCut[HEIGHT][WIDTH];
uint8_t imageBin[HEIGHT][WIDTH];


/**
 * @description: 预锟斤拷锟斤拷锟斤拷锟矫猴拷锟斤拷锟斤拷目前支锟斤拷OTSU锟斤拷蚍ā锟絊AUVOLA锟街诧拷锟斤拷值锟斤拷锟斤拷锟斤拷值锟剿诧拷
 *                  锟斤拷锟斤拷顺锟斤拷 OTSU / 锟斤拷值锟剿诧拷 -> SAUVOLA
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
            //锟斤拷维OTSU锟斤拷占锟斤拷位锟斤拷锟斤拷知锟斤拷锟矫的碉拷锟斤拷
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
 * @description: 锟斤拷蚍ǎ锟饺拷侄锟街碉拷锟斤拷锟斤拷锟皆硷拷锟� -> 锟斤拷锟斤拷锟斤拷图锟斤拷
 * @param {*}
 * @return {uint8_t} th 锟斤拷值锟斤拷锟斤拷值
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
    
    for(int i=0; i<total; ++i)//锟斤拷锟斤拷叶锟街憋拷锟酵�
    {
        histogram[p_pixel[i]]++;
        if(minn>p_pixel[i]) minn = p_pixel[i];
        if(maxn<p_pixel[i]) maxn = p_pixel[i];

    }

    for(int i=minn; i<=min(ConstData.kImageOtsuBrightLimit, maxn); ++i)//锟斤拷取锟揭讹拷权值锟酵ｏ拷使锟斤拷16位锟斤拷锟解爆
    {
        i_mult_histogram[i] = i*histogram[i];
        sum_i_mult_h += i_mult_histogram[i];//锟斤拷锟杰猴拷,锟斤拷锟斤拷之锟斤拷锟斤拷
    }

    for (int i=minn; i<min(ConstData.kImageOtsuBrightLimit, maxn); ++i)//锟斤拷锟斤拷小锟斤拷始锟斤拷锟斤拷锟斤拷寻锟斤拷锟斤拷锟斤拷实锟斤拷锟街�
    {
        //锟斤拷锟斤拷前锟斤拷锟斤拷锟斤拷锟斤拷锟截碉拷锟斤拷锟�
        n0 += histogram[i];
        n1 = total - n0;

        //锟斤拷锟斤拷前锟斤拷锟斤拷锟襟景灰讹拷值锟斤拷
        u0_sum += i_mult_histogram[i];
        u1_sum = sum_i_mult_h - u0_sum;

        u0 = u0_sum / (float)n0;
        u1 = u1_sum / (float)n1;

        n_value = (u0 - u1) * (u0 - u1) * (float)n0 * n1 ;//float锟斤拷前锟斤拷锟斤拷证锟斤拷式转锟斤拷

        if (n_value>max_value)
        {
            max_value = n_value;
            th = i;
        }
    }
    return th;
}

/**
 * @description: sauvola锟街诧拷锟斤拷值锟斤拷锟斤拷锟斤拷锟秸凤拷锟斤拷一锟斤拷锟斤拷原图锟斤拷 -> 锟斤拷锟斤拷锟斤拷图锟斤拷
 * @param {*}
 * @return {*}
 */
void sauvola(void)
{

    int i, j, left, right, down, up, sum, sqsum;
    float mean, v, tmp, area;
    int *integralCols = (int *)malloc(WIDTH * sizeof(int));
    int *integralColsSqu = (int *)malloc(WIDTH * sizeof(int));
    for (int j = 0; j < WIDTH; ++j) //锟斤拷每一锟斤拷预锟斤拷锟斤拷
    {
        integralCols[j] = 0;
        integralColsSqu[j] = 0;
        for (int i = 0; i < kernel_sizeby2_sauvola; ++i) // kernel center 锟斤拷 锟斤拷 kernel_sizeby2_sauvola - 1锟斤拷 之锟斤拷
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
        for (j = 0; j < WIDTH; ++j) //锟斤拷锟斤拷锟斤拷锟矫恳伙拷械拇锟斤拷锟�,锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷郏锟斤拷锟绞硷拷锟斤拷锟酵撅拷锟斤拷锟轿�
        {

            if (up <= 0) //(i <= kernel_sizeby2_sauvola)//锟斤拷始锟斤拷锟街硷拷锟斤拷锟铰讹拷
            {
                // downi = down * WIDTH + j;
                integralCols[j] += mt9v03x_image[down][j];
                integralColsSqu[j] += mt9v03x_image[down][j] * mt9v03x_image[down][j];
            }
            else if (down >= HEIGHT) //(h - 1 - i < kernel_sizeby2_sauvola) //锟斤拷尾锟斤拷锟街硷拷去锟较讹拷
            {
                // upi = (up - 1) * WIDTH + j;
                integralCols[j] -= mt9v03x_image[up - 1][j];
                integralColsSqu[j] -= mt9v03x_image[up - 1][j] * mt9v03x_image[up - 1][j];
            }
            else //锟叫间部锟街硷拷去锟较端硷拷锟斤拷锟铰讹拷
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
        for (j = 0; j < kernel_sizeby2_sauvola; ++j) //锟斤拷锟斤拷预锟斤拷锟斤拷
        {
            sum += integralCols[j];
            sqsum += integralColsSqu[j];
        }
        for (j = 0; j < WIDTH; ++j) //锟斤拷锟津滑达拷
        {
            left = j - kernel_sizeby2_sauvola;
            right = j + kernel_sizeby2_sauvola;
            area = (min(HEIGHT - 1, down) - max(0, up) + 1) * (min(WIDTH - 1, right) - max(0, left) + 1); //锟斤拷锟�
            if (left <= 0)                                                                       //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
            {
                sum += integralCols[right];
                sqsum += integralColsSqu[right];
            }
            else if (right >= WIDTH) //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷也锟�
            {
                sum -= integralCols[left - 1];
                sqsum -= integralColsSqu[left - 1];
            }
            else //锟叫间部锟斤拷
            {
                sum = sum + integralCols[right] - integralCols[left - 1];
                sqsum = sqsum + integralColsSqu[right] - integralColsSqu[left - 1];
            }
            //锟斤拷锟斤拷锟斤拷锟斤拷锟矫达拷锟斤拷目锟斤拷钥锟斤拷锟斤拷锟斤拷锟斤拷业谋锟阶�
            mean = sum / area;
            v = sqsum / area - mean * mean;

            tmp = mt9v03x_image[i][j] + mean * (k_sauvola - 1.0);
            if (tmp <= 0 || tmp * tmp <= k2_sauvola * mean * mean * v / r2_sauvola)
                imageBin[i][j] = 0; //实锟斤拷锟矫碉拷时锟斤拷直锟斤拷锟斤拷01锟斤拷锟斤拷
            else
                imageBin[i][j] = 255;
        }
    }

    free(integralCols);
    free(integralColsSqu);
}


/**
 * @description: sobel锟斤拷缘锟斤拷取锟斤拷锟斤拷锟秸凤拷锟斤拷锟斤拷锟斤拷锟斤拷原图锟斤拷 -> 锟斤拷锟斤拷锟斤拷图锟斤拷
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
 * @description: 锟斤拷值锟剿诧拷锟斤拷原图锟斤拷 -> 原图锟斤拷
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
    tmp = 1;//锟斤拷锟斤拷0锟斤拷锟斤拷锟斤拷
    addtmp = kernel_2_size_medianfilter - kernel_sizeby2_medianfilter*(kernel_sizeby2_medianfilter + 1);
    while(tmp <= N)
    {
        tr[tmp] += addtmp;
        tmp += lowbit(tmp);
    }
    while(cnt!=total)
    {
        ++cnt;//锟斤拷锟斤拷

        //锟斤拷锟铰伙拷锟斤拷锟斤拷锟斤拷
        if(cnt % WIDTH == 1)//锟斤拷锟揭伙拷锟斤拷锟斤拷锟斤拷锟竭猴拷锟斤拷锟铰伙拷锟斤拷
        {
            ++i;
            f = -f;

            up = i - kernel_sizeby2_medianfilter;
            down = i + kernel_sizeby2_medianfilter;
            left = max(j - kernel_sizeby2_medianfilter, 0);
            right = min(j + kernel_sizeby2_medianfilter, WIDTH - 1);
            if(up <= 0)//锟较侧顶锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷虏幔拷喜锟斤拷0锟侥革拷锟斤拷
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
                tmp = 1;//锟斤拷去0锟斤拷锟斤拷锟斤拷
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
                tmp = 1;//锟斤拷锟斤拷0锟斤拷锟斤拷锟斤拷
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
                //锟叫间滑锟斤拷锟斤拷影锟斤拷0锟侥革拷锟斤拷
            }
        }
        else//锟斤拷锟斤拷锟斤拷锟斤拷锟絝值锟斤拷锟揭伙拷锟斤拷锟斤拷f=-1锟襟滑ｏ拷f=1锟揭伙拷
        {

            if(f == 1)//锟斤拷锟揭伙拷锟斤拷
            {
                ++j;
                up = max(i - kernel_sizeby2_medianfilter, 0);
                down = min(i + kernel_sizeby2_medianfilter, HEIGHT - 1);
                left = j - kernel_sizeby2_medianfilter;
                right = j + kernel_sizeby2_medianfilter;
                if(left <= 0)//锟斤拷喽ワ拷锟斤拷锟斤拷锟斤拷锟斤拷锟揭侧，锟斤拷锟斤拷0锟侥革拷锟斤拷
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
                    tmp = 1;//锟斤拷去0锟斤拷锟斤拷锟斤拷
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
                    tmp = 1;//锟斤拷锟斤拷0锟斤拷锟斤拷锟斤拷
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] += addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else//锟叫间滑锟斤拷锟斤拷影锟斤拷0锟侥革拷锟斤拷
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
                if(left < 0)//锟斤拷喽ワ拷锟斤拷锟斤拷锟斤拷去锟揭侧，锟斤拷锟斤拷0锟侥革拷锟斤拷
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
                    tmp = 1;//锟斤拷锟斤拷0锟斤拷锟斤拷锟斤拷
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] += addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else if(right >= WIDTH - 1)//锟揭侧顶锟斤拷锟斤拷锟斤拷锟斤拷啵拷锟饺ワ拷也锟�0锟侥革拷锟斤拷
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
                    tmp = 1;//锟斤拷锟斤拷0锟斤拷锟斤拷锟斤拷
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] -= addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else//锟叫间滑锟斤拷锟斤拷影锟斤拷0锟侥革拷锟斤拷
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
        //锟斤拷锟铰斤拷锟斤拷
        //锟斤拷取锟斤拷位锟斤拷锟斤拷值

        sum = 0; ret = 0;
        for(int t = 8; ~t; --t)
        {
            ret += 1 << t;                      // 锟斤拷锟斤拷锟斤拷展
            if (ret >= N || sum + tr[ret] >= kernel_mid_medianfilter)  // 锟斤拷锟斤拷锟秸故э拷锟�
                ret -= 1 << t;
            else
                sum += tr[ret];  // 锟斤拷展锟缴癸拷锟斤拷 要锟斤拷锟斤拷之前锟斤拷偷锟街�
        }
        temp[i][j] = ret;
        //printf("%d %d %d %u\n", i, j, ret, tr[1]);
    }
    memcpy(mt9v03x_image,temp,sizeof(temp));
}


/**
 * @description: 锟斤拷蚀
 * @param {*}
 * @return {*}
 * 太锟斤拷锟斤拷, 锟斤拷锟角撅拷锟斤拷锟斤拷锟斤拷,锟斤拷锟斤拷只锟斤拷要锟剿筹拷锟斤拷锟斤拷锟较的黑碉拷,锟斤拷锟斤拷围只要锟阶碉拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷
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
 * @description: 锟斤拷锟斤拷
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
 * @description: 锟斤拷锟斤拷锟姐，锟饺革拷蚀锟斤拷锟斤拷锟斤拷
 * @param {*}
 * @return {*}
 */
void morph_open(void)
{
    morph_erosion();
    morph_dilition();
}

/**
 * @description: 锟斤拷锟斤拷锟姐，锟斤拷锟斤拷锟酵猴拷蚀
 * @param {*}
 * @return {*}
 */
void morph_close(void)
{
    morph_dilition();
    morph_erosion();
}
