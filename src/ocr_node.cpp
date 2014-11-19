#include <string>
#include <iostream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ocr_rov/Compass.h>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>

#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

Mat mod[10];

ros::Publisher compass_publisher;
image_transport::Publisher image_publisher;

void carregaModelo()
{
    //cout << "Templates:" << endl;
    char nome[100];
    for(int i = 0 ; i < 10; i++)
    {
        sprintf(nome, "modelos/%d.pbm", i);
        mod[i] = imread(nome, 0);
    }
}

int match(Mat img, Mat tmp)
{
    int x, y, xl , yl,
    rl = img.rows - tmp.rows,
    rc = img.cols - tmp.cols;
    int soma = 0, maiorSoma = 0;


    for(y = 0 ; y < rl; y++)
    {
        for(x = 0; x < rc; x++)
        {
            soma = 0;
            for(yl = 0; yl < tmp.rows ; yl++)
            {
                for(xl = 0 ; xl < tmp.cols; xl++)
                {
                    if(img.at<unsigned char>(y+yl, x+xl) == tmp.at<unsigned char>(yl, xl))
                        soma++;
                }
            }
            if(soma > maiorSoma)
            {
                maiorSoma = soma;
            }
        }
    }
    return maiorSoma;
}

double compare(Mat img, Mat tmp)
{
    // Criando a matriz de resultados do matching
    Mat res;
    int resCol = img.cols - tmp.cols + 1;
    int resLin = img.rows - tmp.rows + 1;

    res.create(resCol, resLin, CV_32FC1);

    // Faz o matching e normaliza
    matchTemplate( img, tmp, res,  CV_TM_CCORR_NORMED );
//    normalize( res, res, 0, 1, NORM_MINMAX, -1, Mat() );

    double minVal, maxVal;
    minMaxLoc( res, &minVal, &maxVal);

    return maxVal;
}

int valorCompass(int pos, int leitura){
    int b = 0;
    switch (pos) {
    case 0:
        b += leitura*100;
        break;
    case 1:
        b += leitura*10;
        break;
    case 2:
        b += leitura;
        break;
    default:
        b += 0;
        break;
    }
    return b;
}

void imageCallback(const sensor_msgs::ImageConstPtr &image)
{

    cv_bridge::CvImageConstPtr img = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::MONO8);

    int i = 0, angle = 0;
    Mat img_bin;
    Mat img_croped;
    char nome[100];
    ocr_rov::Compass msg;


    // Binariza
//    adaptiveThreshold(img,bin, 220, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 0.1);
//    adaptiveThreshold(img,bin, 220, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 7, 2.0);
    threshold(img->image,img_bin,210,255,THRESH_BINARY_INV);

    float xIni= 245, yIni= 388, lar=15, alt=20, folgaX = 4, folgaY = 4;  // Linha cima  // Folga para o template matching

    xIni -= folgaX/2.f;  yIni-= folgaY/2.f;


    vector<int> param;
    param.push_back(CV_IMWRITE_PXM_BINARY);

    for(i = 0 ; i < 3 ; i++)//i<3 Pega so o valor da bussola, Obs: para a linha de cima i<20 e para a de baixo tmb i<40
    {
        if(i == 20) // Passal para a linha de baixo
        {
            xIni= 245;
            yIni= 415;
            lar=15;
            alt=20;
        }
        Rect r(xIni , yIni , lar + folgaX ,alt + folgaY);

        img_croped = img_bin(r);  //cout << "corte l= " << corte.rows << " c=" << corte.cols << endl;

        // Com a nossa implementacao de template matching
        int k = 0, max = 0;
        for(int j = 0 ; j < 10; j++)
        {
            int x = match(img_croped, mod[j]);
            if(max < x)
            {
                max = x;
                k = j;
            }
        }

        if(k>0 && k<10)
            angle += valorCompass(i, k);

        xIni += lar + 0.4f;
        r.x = xIni;
    }
    printf("\n");

    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    msg.angle = angles::from_degrees(angle);

    publishBynaryImage(img_bin);
    compass_publisher.publish(msg);

}

void publishBynaryImage(Mat &img)
{
    cv_bridge::CvImage image;

    image.header.stamp = ros::Time::now();
    image.header.frame_id = "world";

    image.encoding = sensor_msgs::image_encodings::MONO8;

    image.image = img;

    image_publisher.publish(image.toImageMsg());

}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"ocr_rov");

    ros::NodeHandle nd;

    image_transport::ImageTransport it(nd);
    image_transport::Subscriber image_subscriber;


    //! hint to modify the image_transport. Here I use raw transport
    image_transport::TransportHints hints("raw",ros::TransportHints(),nd);

    //! image subscription
    image_subscriber = it.subscribe("/image_raw",1,&imageCallback,hints);

    image_publisher = it.advertise("/image_binary", 1);

    compass_publisher = nd.advertise<ocr_rov::Compass>("compass",100);

    carregaModelo();

    ros::spin();

    return 0;
}
