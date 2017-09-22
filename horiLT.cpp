#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <cstdarg>
#include "opencv2/opencv.hpp"
#include "fstream"
#include "Ctracker.h"
#include <dirent.h>
#include <math.h>
#include <time.h>
#include	<opencv2/videostab.hpp>
#include 	<opencv2/features2d.hpp>
#include <opencv2/videostab/global_motion.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;
using namespace cv;
using namespace	cv::videostab;

Mat horizon_cont_original(Mat img)
{
	resize(img,img,Size(640,360));

Mat lowhalfremove = Mat::zeros(img.size(),CV_8UC1);
lowhalfremove(Rect(0,0,img.cols,img.rows/2)) = 1;
	
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size(2,2) );
	Mat diamond = Mat(5,5,CV_8U,Scalar(1));
	diamond.at<uchar>(0,0)= 0;
    diamond.at<uchar>(0,1)= 0;
	diamond.at<uchar>(1,0)= 0;
	diamond.at<uchar>(4,4)= 0;
	diamond.at<uchar>(3,4)= 0;
	diamond.at<uchar>(4,3)= 0;
	diamond.at<uchar>(4,0)= 0;
	diamond.at<uchar>(4,1)= 0;
	diamond.at<uchar>(3,0)= 0;
	diamond.at<uchar>(0,4)= 0;
	diamond.at<uchar>(0,3)= 0;
	diamond.at<uchar>(1,4)= 0;

vector<Mat> imgl(3);

Mat imgopen, imgclos;
morphologyEx(img,imgopen,MORPH_OPEN,element);
morphologyEx(imgopen,imgclos,MORPH_CLOSE,element);

Mat temefr1,temefr2,tframe;
dilate( imgclos,temefr1, diamond);
morphologyEx( imgclos,temefr2, MORPH_CLOSE, diamond);

tframe = temefr1 - temefr2;
	
dilate(tframe,tframe,diamond);
dilate(tframe,tframe,diamond);
dilate(tframe,tframe,diamond);
dilate(tframe,tframe,diamond);
dilate(tframe,tframe,diamond);

	vector<Mat> imm(3);
	split(tframe,imm);
	
multiply(imm[0],lowhalfremove,imm[0]);				// Assumption:: Horizon wont exist below the vertical midpoint of the image
multiply(imm[1],lowhalfremove,imm[1]);				// Hence lower half removed
multiply(imm[2],lowhalfremove,imm[2]);
	
	Mat t1,t2,t3;
	threshold(imm[0],t1,11,255, THRESH_BINARY | THRESH_OTSU);   
	threshold(imm[1],t2,12,255, THRESH_BINARY | THRESH_OTSU);   
	threshold(imm[2],t3,13,255, THRESH_BINARY | THRESH_OTSU);   
		
	bitwise_and(t1,t2,t2);
	bitwise_and(t2,t3,t3);
	
	morphologyEx(t3,t3,MORPH_OPEN,Mat());
	morphologyEx(t3,t3,MORPH_CLOSE,Mat());
	
	dilate(t3,t3,Mat());
	dilate(t3,t3,Mat());
	dilate(t3,t3,Mat());
	
	erode(t3,t3,Mat());
	erode(t3,t3,Mat());
	erode(t3,t3,Mat());
	
	Mat Mask = Mat::zeros(t3.size(),CV_8UC1);
	vector<vector<Point> > newcontours;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(t3, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
	int kdivision = 6;
	int *maxx = NULL, *maxy = NULL;
	maxx = new int[kdivision];
	maxy = new int[kdivision];
	
	for(int i = 0; i< kdivision;i++) {maxx[i] = 0; maxy[i] = 0;}
	
	for (int k = 0; k< kdivision; k++)
	{
	
	int A = 0,B = 0;
	A = (t3.cols / kdivision) * (k);
	B = (t3.cols / kdivision) * (k+1);
		
		
	for(int i = 0; i< contours.size();i++)
	{
		for(int j = 0; j< contours[i].size();j++)
		{
			/*
			if( ( contours[i][j].x < t3.cols / 2)&&(contours[i][j].y < t3.rows / 2) )		// For accessing only the first quadrant
			{
				if(contours[i][j].y > maxy1)
				{
				maxy1 = contours[i][j].y;
				maxx1 = contours[i][j].x;
				}
			
			}
			
			if( (contours[i][j].x > t3.cols / 2)&&( contours[i][j].y < t3.rows / 2) )		// For accessing only the second quadrant
			{
			
				if(contours[i][j].y > maxy2)
				{
				maxy2 = contours[i][j].y;
				maxx2 = contours[i][j].x;
				}
			
			}
		*/
		
			if( ( contours[i][j].x > A ) && ( contours[i][j].x < B) && (contours[i][j].y < t3.rows / 2) )
			{
					if(contours[i][j].y > maxy[k])
					{
					maxx[k] = contours[i][j].x;
					maxy[k] = contours[i][j].y;
					}
			}
			
			
		}
	}
	
	
	}
	/*	for(int i = 0; i< contours.size();i++)
	{
	vector<Point> tmppoint;
		
		for(int j = 0; j< contours[i].size();j++)
		{
		
			if( ( contours[i][j].x > maxx1 )&&(contours[i][j].x < maxx2) )		// For accessing only the first quadrant
			{
				tmppoint.push_back(contours[i][j]);
			
			}
			
						
		}
		
		
		
		newcontours.push_back(tmppoint);
		
	} */
	
	// For finding contours
	vector<Point> tmp;
	tmp.push_back(Point( t3.cols - 1 , 0 ));
	tmp.push_back(Point(0,0));
	tmp.push_back(Point(0,maxy[0]));
	tmp.push_back( Point(maxx[0],maxy[0]) );
	for(int u = 1; u < kdivision; u++)
	{	
//	line(Mask,Point(maxx[u-1],maxy[u-1]),Point(maxx[u],maxy[u]),Scalar(255),1,8,0);
	tmp.push_back(Point(maxx[u],maxy[u]));
	}
	tmp.push_back(Point(t3.cols - 1,maxy[kdivision - 1]));
	
//	line(Mask,Point(0,maxy[0]),Point(maxx[0],maxy[0]),Scalar(255),1,8,0);
//	line(Mask,Point(t3.cols - 1,maxy[kdivision - 1]),Point(maxx[kdivision - 1],maxy[kdivision - 1]),Scalar(255),1,8,0);
	
	newcontours.push_back(tmp);
	drawContours(Mask, newcontours, -1, Scalar::all(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
	bitwise_not(Mask,Mask);
	
	vector<Mat> imggg;
	split(img,imggg);
	bitwise_and(imggg[0],Mask,imggg[0]);
	bitwise_and(imggg[1],Mask,imggg[1]);
	bitwise_and(imggg[2],Mask,imggg[2]);
	merge(imggg,img);
	
	delete [] maxx;
	delete [] maxy;
	
return img;

}

int ncount(int A, int B, vector< vector<Point> > vect,int R)
{
int count = 0;
	
	for(int i = 0; i< vect.size(); i++)
	{
		for(int j = 0; j<vect[i].size();j++)
		{
		
		if( (vect[i][j].x > A) &&  (vect[i][j].x < B) && (vect[i][j].y < (R/2)) )
		{
		count++;
		}
		
		}
	}
	
return count;	
}

Mat horizon_cont(Mat img,int kdivision = 6)			// New and improved horizon cont
{
	resize(img,img,Size(640,360));

Mat lowhalfremove = Mat::zeros(img.size(),CV_8UC1);
lowhalfremove(Rect(0,0,img.cols,img.rows/2)) = 1;
	
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size(2,2) );
	Mat diamond = Mat(5,5,CV_8U,Scalar(1));
	diamond.at<uchar>(0,0)= 0;
    diamond.at<uchar>(0,1)= 0;
	diamond.at<uchar>(1,0)= 0;
	diamond.at<uchar>(4,4)= 0;
	diamond.at<uchar>(3,4)= 0;
	diamond.at<uchar>(4,3)= 0;
	diamond.at<uchar>(4,0)= 0;
	diamond.at<uchar>(4,1)= 0;
	diamond.at<uchar>(3,0)= 0;
	diamond.at<uchar>(0,4)= 0;
	diamond.at<uchar>(0,3)= 0;
	diamond.at<uchar>(1,4)= 0;

vector<Mat> imgl(3);

Mat imgopen, imgclos;
morphologyEx(img,imgopen,MORPH_OPEN,element);
morphologyEx(imgopen,imgclos,MORPH_CLOSE,element);

Mat temefr1,temefr2,tframe;
dilate( imgclos,temefr1, diamond);
morphologyEx( imgclos,temefr2, MORPH_CLOSE, diamond);

tframe = temefr1 - temefr2;
	
dilate(tframe,tframe,diamond);
dilate(tframe,tframe,diamond);
dilate(tframe,tframe,diamond);
dilate(tframe,tframe,diamond);
dilate(tframe,tframe,diamond);

	vector<Mat> imm(3);
	split(tframe,imm);
	
multiply(imm[0],lowhalfremove,imm[0]);				// Assumption:: Horizon wont exist below the vertical midpoint of the image
multiply(imm[1],lowhalfremove,imm[1]);				// Hence lower half removed
multiply(imm[2],lowhalfremove,imm[2]);
	
	Mat t1,t2,t3;
	threshold(imm[0],t1,11,255, THRESH_BINARY | THRESH_OTSU);   
	threshold(imm[1],t2,12,255, THRESH_BINARY | THRESH_OTSU);   
	threshold(imm[2],t3,13,255, THRESH_BINARY | THRESH_OTSU);   
		
	bitwise_and(t1,t2,t2);
	bitwise_and(t2,t3,t3);
	
	morphologyEx(t3,t3,MORPH_OPEN,Mat());
	morphologyEx(t3,t3,MORPH_CLOSE,Mat());
	
	dilate(t3,t3,Mat());
	dilate(t3,t3,Mat());
	dilate(t3,t3,Mat());
	
	erode(t3,t3,Mat());
	erode(t3,t3,Mat());
	erode(t3,t3,Mat());
	
	Mat Mask = Mat::zeros(t3.size(),CV_8UC1);
	vector<vector<Point> > newcontours;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(t3, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	int *maxx = NULL, *maxy = NULL;
	maxx = new int[kdivision];
	maxy = new int[kdivision];
	
	for(int i = 0; i< kdivision;i++) {maxx[i] = 0; maxy[i] = 0;}
	
	for (int k = 0; k< kdivision; k++)
	{
	
	int A = 0,B = 0;
	A = (t3.cols / kdivision) * (k);
	B = (t3.cols / kdivision) * (k+1);
		
				
			if(ncount(A,B,contours,t3.rows))						// If there are points present between A and B process normally
			{
	for(int i = 0; i< contours.size();i++)
	{
		for(int j = 0; j< contours[i].size();j++)
		{
			/*
			if( ( contours[i][j].x < t3.cols / 2)&&(contours[i][j].y < t3.rows / 2) )		// For accessing only the first quadrant
			{
				if(contours[i][j].y > maxy1)
				{
				maxy1 = contours[i][j].y;
				maxx1 = contours[i][j].x;
				}
			
			}
			
			if( (contours[i][j].x > t3.cols / 2)&&( contours[i][j].y < t3.rows / 2) )		// For accessing only the second quadrant
			{
			
				if(contours[i][j].y > maxy2)
				{
				maxy2 = contours[i][j].y;
				maxx2 = contours[i][j].x;
				}
			
			}
		*/

			
			if( ( contours[i][j].x > A ) && ( contours[i][j].x < B) && (contours[i][j].y < t3.rows / 2) )
			{
					if(contours[i][j].y > maxy[k])
					{
					maxx[k] = contours[i][j].x;
					maxy[k] = contours[i][j].y;
					}
			}
			
	
		}
	}
	
			}
			
			else
			{
			Point nearestpt = contours[0][0];
			int minval = std::pow(std::pow(contours[0][0].x - A,2),0.5);		// can replace by std::pow(std::pow(X,2),0.5);
			int miny = nearestpt.y;
				
				for(int pp = 0; pp < contours.size(); pp++)				// contours may have only one value :: WARNING
				{
					for(int l = 0; l< contours[pp].size(); l++)
					{
					int diff = std::pow(std::pow(contours[pp][l].x - A,2),0.5);
					int ytmp = contours[pp][l].y;
						
					if( (minval > diff) && (ytmp > miny) && (contours[pp][l].y < (t3.rows / 2)) )
					{
					minval = diff;
					nearestpt = contours[pp][l];
					miny = ytmp;	
					}
					
					}
				}			// have nearest point closest to A via the X coordinate 
			maxx[k] = A;
			maxy[k] = nearestpt.y;
				
			}
	}
	/*	for(int i = 0; i< contours.size();i++)
	{
	vector<Point> tmppoint;
		
		for(int j = 0; j< contours[i].size();j++)
		{
		
			if( ( contours[i][j].x > maxx1 )&&(contours[i][j].x < maxx2) )		// For accessing only the first quadrant
			{
				tmppoint.push_back(contours[i][j]);
			
			}
			
						
		}
		
		
		
		newcontours.push_back(tmppoint);
		
	} */
	
	// For finding contours
	vector<Point> tmp;
	tmp.push_back(Point( t3.cols - 1 , 0 ));
	tmp.push_back(Point(0,0));
	tmp.push_back(Point(0,maxy[0]));
	tmp.push_back( Point(maxx[0],maxy[0]) );
	
	for(int u = 1; u < kdivision; u++)
	{	
//	line(Mask,Point(maxx[u-1],maxy[u-1]),Point(maxx[u],maxy[u]),Scalar(255),1,8,0);
	tmp.push_back(Point(maxx[u],maxy[u]));
	}
	tmp.push_back(Point(t3.cols - 1,maxy[kdivision - 1]));
	
//	line(Mask,Point(0,maxy[0]),Point(maxx[0],maxy[0]),Scalar(255),1,8,0);
//	line(Mask,Point(t3.cols - 1,maxy[kdivision - 1]),Point(maxx[kdivision - 1],maxy[kdivision - 1]),Scalar(255),1,8,0);
	
	newcontours.push_back(tmp);
	drawContours(Mask, newcontours, -1, Scalar::all(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
	bitwise_not(Mask,Mask);
	
	vector<Mat> imggg;
	split(img,imggg);
	bitwise_and(imggg[0],Mask,imggg[0]);
	bitwise_and(imggg[1],Mask,imggg[1]);
	bitwise_and(imggg[2],Mask,imggg[2]);
	merge(imggg,img);
	
	delete [] maxx;
	delete [] maxy;
	
return img;

}

int main()
{
	/*
    Mat img = imread("frame71.jpeg");
	Mat horri = horizon_cont(img);
	imshow("Horizon Made",horri);
	waitKey();
	*/
	
	Mat img;
	VideoCapture cap;
	cap.open("dolv.mp4");
	
	VideoWriter wir;
	int codec = CV_FOURCC('M','J','P','G');
	wir.open("dolv(1)_horizon_removed.avi",codec ,cap.get(CV_CAP_PROP_FPS),Size( 640,360 ));
	
	while( cap.get(CV_CAP_PROP_POS_FRAMES) < cap.get(CV_CAP_PROP_FRAME_COUNT) )
	{

	cap>>img;
	Mat horri = horizon_cont(img);	
	wir<<horri;		
	}

	cout<<"\n Done!!!\n";
return 1;
}