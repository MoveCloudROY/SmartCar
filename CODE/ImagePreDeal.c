/*
 * @Author: ROY1994
 * @Date: 2022-02-04 13:40:22
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:40:52
 * @FilePath: \myImageDeal_v0.1\ImagePreDeal.cpp
 * @Description: ���Ԥ��������
 */

#include "ImagePreDeal.h"

extern ConstDataTypeDef ConstData;
extern uint8_t mt9v03x_image[120][188];
uint8_t imageCut[HEIGHT][WIDTH];
uint8_t imageBin[HEIGHT][WIDTH];


/**
 * @description: Ԥ�������ú�����Ŀǰ֧��OTSU��򷨡�SAUVOLA�ֲ���ֵ������ֵ�˲�
 *                  ����˳�� OTSU / ��ֵ�˲� -> SAUVOLA
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
            //��άOTSU��ռ��λ����֪���õĵ���
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
 * @description: ��򷨣�ȫ�ֶ�ֵ������ԭͼ�� -> ������ͼ��
 * @param {*}
 * @return {uint8_t} th ��ֵ����ֵ
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
    
    for(int i=0; i<total; ++i)//����Ҷ�ֱ��ͼ
    {
        histogram[p_pixel[i]]++;
        if(minn>p_pixel[i]) minn = p_pixel[i];
        if(maxn<p_pixel[i]) maxn = p_pixel[i];

    }

    for(int i=minn; i<=min(ConstData.kImageOtsuBrightLimit, maxn); ++i)//��ȡ�Ҷ�Ȩֵ�ͣ�ʹ��16λ���ⱬ
    {
        i_mult_histogram[i] = i*histogram[i];
        sum_i_mult_h += i_mult_histogram[i];//���ܺ�,����֮����
    }

    for (int i=minn; i<min(ConstData.kImageOtsuBrightLimit, maxn); ++i)//����С��ʼ������Ѱ������ʵ���ֵ
    {
        //����ǰ���������ص����
        n0 += histogram[i];
        n1 = total - n0;
        
        //����ǰ�����󾰻Ҷ�ֵ��
        u0_sum += i_mult_histogram[i];
        u1_sum = sum_i_mult_h - u0_sum;

        u0 = u0_sum / (float)n0;
        u1 = u1_sum / (float)n1;
        
        n_value = (u0 - u1) * (u0 - u1) * (float)n0 * n1 ;//float��ǰ����֤��ʽת��

        if (n_value>max_value)
        {
            max_value = n_value;
            th = i;
        }
    }
    return th;
}

/**
 * @description: sauvola�ֲ���ֵ�������շ���һ����ԭͼ�� -> ������ͼ��
 * @param {*}
 * @return {*}
 */
void sauvola(void)
{

    int i, j, left, right, down, up, sum, sqsum;
    float mean, v, tmp, area;
    int *integralCols = (int *)malloc(WIDTH * sizeof(int));
    int *integralColsSqu = (int *)malloc(WIDTH * sizeof(int));
    for (int j = 0; j < WIDTH; ++j) //��ÿһ��Ԥ����
    {
        integralCols[j] = 0;
        integralColsSqu[j] = 0;
        for (int i = 0; i < kernel_sizeby2_sauvola; ++i) // kernel center �� �� kernel_sizeby2_sauvola - 1�� ֮��
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
        for (j = 0; j < WIDTH; ++j) //�������ÿһ�еĴ���,������������ۣ���ʼ����;����β
        {

            if (up <= 0) //(i <= kernel_sizeby2_sauvola)//��ʼ���ּ����¶�
            {
                // downi = down * WIDTH + j;
                integralCols[j] += mt9v03x_image[down][j];
                integralColsSqu[j] += mt9v03x_image[down][j] * mt9v03x_image[down][j];
            }
            else if (down >= HEIGHT) //(h - 1 - i < kernel_sizeby2_sauvola) //��β���ּ�ȥ�϶�
            {
                // upi = (up - 1) * WIDTH + j;
                integralCols[j] -= mt9v03x_image[up - 1][j];
                integralColsSqu[j] -= mt9v03x_image[up - 1][j] * mt9v03x_image[up - 1][j];
            }
            else //�м䲿�ּ�ȥ�϶˼����¶�
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
        for (j = 0; j < kernel_sizeby2_sauvola; ++j) //����Ԥ����
        {
            sum += integralCols[j];
            sqsum += integralColsSqu[j];
        }
        for (j = 0; j < WIDTH; ++j) //���򻬴�
        {
            left = j - kernel_sizeby2_sauvola;
            right = j + kernel_sizeby2_sauvola;
            area = (min(HEIGHT - 1, down) - max(0, up) + 1) * (min(WIDTH - 1, right) - max(0, left) + 1); //���
            if (left <= 0)                                                                       //������������
            {
                sum += integralCols[right];
                sqsum += integralColsSqu[right];
            }
            else if (right >= WIDTH) //����������Ҳ�
            {
                sum -= integralCols[left - 1];
                sqsum -= integralColsSqu[left - 1];
            }
            else //�м䲿��
            {
                sum = sum + integralCols[right] - integralCols[left - 1];
                sqsum = sqsum + integralColsSqu[right] - integralColsSqu[left - 1];
            }
            //���������ô���Ŀ��Կ��������ҵı�ע
            mean = sum / area;
            v = sqsum / area - mean * mean;

            tmp = mt9v03x_image[i][j] + mean * (k_sauvola - 1.0);
            if (tmp <= 0 || tmp * tmp <= k2_sauvola * mean * mean * v / r2_sauvola)
                imageBin[i][j] = 0; //ʵ���õ�ʱ��ֱ����01����
            else
                imageBin[i][j] = 255;
        }
    }

    free(integralCols);
    free(integralColsSqu);
}


/**
 * @description: sobel��Ե��ȡ�����շ���������ԭͼ�� -> ������ͼ��
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
 * @description: ��ֵ�˲���ԭͼ�� -> ԭͼ��
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
    tmp = 1;//����0������
    addtmp = kernel_2_size_medianfilter - kernel_sizeby2_medianfilter*(kernel_sizeby2_medianfilter + 1);
    while(tmp <= N)
    {
        tr[tmp] += addtmp;
        tmp += lowbit(tmp);
    }
    while(cnt!=total)
    {
        ++cnt;//����

        //���»�������
        if(cnt % WIDTH == 1)//���һ��������ߺ����»���
        {
            ++i;
            f = -f;

            up = i - kernel_sizeby2_medianfilter;
            down = i + kernel_sizeby2_medianfilter;
            left = max(j - kernel_sizeby2_medianfilter, 0);
            right = min(j + kernel_sizeby2_medianfilter, WIDTH - 1);
            if(up <= 0)//�ϲඥ����������²ᣬ�ϲ��0�ĸ���
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
                tmp = 1;//��ȥ0������
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
                tmp = 1;//����0������
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
                //�м们����Ӱ��0�ĸ���
            }
        }
        else//���������fֵ���һ�����f=-1�󻬣�f=1�һ�
        {

            if(f == 1)//���һ���
            {
                ++j;
                up = max(i - kernel_sizeby2_medianfilter, 0);
                down = min(i + kernel_sizeby2_medianfilter, HEIGHT - 1);
                left = j - kernel_sizeby2_medianfilter;
                right = j + kernel_sizeby2_medianfilter;
                if(left <= 0)//��ඥ����������Ҳ࣬����0�ĸ���
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
                    tmp = 1;//��ȥ0������
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
                    tmp = 1;//����0������
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] += addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else//�м们����Ӱ��0�ĸ���
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
                if(left < 0)//��ඥ�������ȥ�Ҳ࣬����0�ĸ���
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
                    tmp = 1;//����0������
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] += addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else if(right >= WIDTH - 1)//�Ҳඥ��������࣬��ȥ�Ҳ�0�ĸ���
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
                    tmp = 1;//����0������
                    addtmp = down - up + 1;
                    while(tmp <= N)
                    {
                        tr[tmp] -= addtmp;
                        tmp += lowbit(tmp);
                    }
                }
                else//�м们����Ӱ��0�ĸ���
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
        //���½���
        //��ȡ��λ����ֵ

        sum = 0; ret = 0;
        for(int t = 8; ~t; --t)
        {
            ret += 1 << t;                      // ������չ
            if (ret >= N || sum + tr[ret] >= kernel_mid_medianfilter)  // �����չʧ��
                ret -= 1 << t;
            else
                sum += tr[ret];  // ��չ�ɹ��� Ҫ����֮ǰ��͵�ֵ
        }
        temp[i][j] = ret;
        //printf("%d %d %d %u\n", i, j, ret, tr[1]);
    }
    memcpy(mt9v03x_image,temp,sizeof(temp));
}


/**
 * @description: ��ʴ
 * @param {*}
 * @return {*}
 * ̫����, ���Ǿ�������,����ֻ��Ҫ�˳������ϵĺڵ�,����ΧֻҪ�׵�����������ֵ����
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
 * @description: ����
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
 * @description: �����㣬�ȸ�ʴ������
 * @param {*}
 * @return {*}
 */
void morph_open(void)
{
    morph_erosion();
    morph_dilition();
}

/**
 * @description: �����㣬�����ͺ�ʴ
 * @param {*}
 * @return {*}
 */
void morph_close(void)
{
    morph_dilition();
    morph_erosion();
}
