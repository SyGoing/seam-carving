#pragma once
#include <opencv2/opencv.hpp>
#include<vector>
#include<memory>
#include<algorithm>
using namespace cv;

///default is Vertical mode,if horizontal just transpose
///seam carving :resize smaller finished; enlarge to do 
//author:SyGoing
//date:2019.07.01

typedef struct Node {
	float value;
	int path;
}node;

int which_min2(float x, float y);
int which_min3(float x, float y, float z);


void calc_energy(cv::Mat &image,cv::Mat &original_energy);
void calc_cumulative_energy(cv::Mat &original_energy, std::vector<node> &table);
void find_seam(std::vector<node> &table,std::vector<int> &path,int w,int h);
void remove_seam(cv::Mat &image, std::vector<int>&path, cv::Mat &out);

///first test   //only one direction
//direction, 
//0:horizontal 
//1:vertical
void seam_carving_single(cv::Mat &image, cv::Mat &output,int direction=0);


//the energy of the seam（vertical or horizontal)
int compute_seam_cost(std::vector<node> &table, cv::Size &size_,std::vector<int> &path, int type_);

void seamEngine(cv::Mat &image, cv::Mat &output,int reduce_col,int reduce_row);
void seamEngine(cv::Mat &image, cv::Mat &output, float scale_w, float scale_h);
