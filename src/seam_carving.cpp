#include "seam_carving.hpp"



int which_min2(float x, float y)
{
	return x < y ? 0 : 1;
}

int which_min3(float x, float y, float z)
{
	int min_id = -1;
	float min_value = 9999;
	float values[3] = { x,y,z };
	for (int i = 0; i < 3; i++) {
		if (values[i] < min_value) {
			min_value = values[i];
			min_id = i;
		}
	}
	return min_id;
}

void calc_energy(cv::Mat & image, cv::Mat &original_energy)
{
	
	Mat dx, dy,abs_dx,abs_dy, gray;
	cvtColor(image, gray, cv::COLOR_BGR2GRAY);

	Sobel(gray, dx, CV_64F, 1, 0);
	Sobel(gray, dy, CV_64F, 0, 1);

	convertScaleAbs(dx, abs_dx);
	convertScaleAbs(dy, abs_dy);
	addWeighted(abs_dx, 0.5, abs_dy, 0.5, 0, original_energy);
	original_energy.convertTo(original_energy, CV_8UC1);


	//初始能量图
	namedWindow("grad", CV_WINDOW_NORMAL);
	imshow("grad", original_energy);
	waitKey(1);


	//Mat dx, dy;
	//Sobel(image, dx, CV_64F, 1, 0);
	//Sobel(image, dy, CV_64F, 0, 1);
	//magnitude(dx, dy, output);

	//double min_value, max_value, Z;
	//minMaxLoc(output, &min_value, &max_value);
	//Z = 1 / max_value * 255;
	//output = output * Z;                    //normalize
	//output.convertTo(output, CV_8U);
}

void calc_cumulative_energy(cv::Mat &original_energy, std::vector<node> &table)
{
	int rows = original_energy.rows;
	int cols = original_energy.cols;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			int id = -1;
			if (i == 0) {
				table[i*cols + j].value = original_energy.at<uchar>(i, j);
				table[i*cols + j].path = -6;
			}
			else {
				if (j == 0) {
					id = which_min2(original_energy.at<uchar>(i-1, j), original_energy.at<uchar>(i-1, j+1));
					table[i*cols + j].value = original_energy.at<uchar>(i, j) + original_energy.at<uchar>(i - 1, j+id);
					table[i*cols + j].path = id+1;
				}
				else if (j == cols - 1) {
					id= which_min2(original_energy.at<uchar>(i-1,j-1), original_energy.at<uchar>(i - 1, j ));
					table[i*cols + j].value = original_energy.at<uchar>(i, j) + original_energy.at<uchar>(i-1, j-1+id);
					table[i*cols + j].path = id;
				}
				else {
					id = which_min3(original_energy.at<uchar>(i - 1, j - 1), original_energy.at<uchar>(i - 1, j), original_energy.at<uchar>(i - 1, j + 1));
					table[i*cols + j].value = original_energy.at<uchar>(i, j) + original_energy.at<uchar>(i - 1, j - 1 + id);
					table[i*cols + j].path = id;
				}
			}
		}
	}
}

void find_seam(std::vector<node>& table,std::vector<int> &path, int w, int h)
{
	//path size is h-1
	int min_index;
	float min_energy = 999999;
	for (int i = 0; i < w; i++) {
		if (table[(h - 1)*w + i].value < min_energy) {
			min_index = i;
			min_energy = table[(h - 1)*w + i].value;
		}
	}

	int tmp_id;
	int count = 0;
	path[h - 1] = min_index;
	tmp_id = min_index;
	for (int i = h - 1; i >0; i--) {
		tmp_id = table[i*w + tmp_id].path - 1 + tmp_id;
		path[i - 1] = tmp_id;
	}
	
}

void remove_seam(cv::Mat & image, std::vector<int> &path, cv::Mat & out)
{
	cv::Mat tmp = image.clone();
	for (int i = 0; i < image.rows; i++) {
		Vec3b values(0, 0, 255);
		tmp.at<Vec3b>(i, path[i]) = values;
	}
	namedWindow("seam", CV_WINDOW_NORMAL);
	imshow("seam", tmp);
	waitKey(1);

	//直接移除
	//移除后-接缝处均值光滑处理
	for (int r = 0; r < image.rows; r++) {
		for (int c = 0; c < image.cols; c++) {
			if (c >= path[r])
				out.at<Vec3b>(r, c) = image.at<Vec3b>(r, c + 1);
			else
				out.at<Vec3b>(r, c) = image.at<Vec3b>(r, c);
		}
	}
}

void seam_carving_single(cv::Mat & image, cv::Mat & output, int direction)
{
	//0 horizontal 
	//1 vertical
	if (direction == 0) {
		transpose(image, image);
        flip(image, image, 1); //transpose+flip(1)=CW 

		//cv::namedWindow("trans", CV_WINDOW_NORMAL);
		//cv::imshow("trans", image);
		//cv::waitKey();
	}


	cv::Mat energy;
	
	int cols = image.cols;
	int rows = image.rows;
	cv::Mat tmpout(rows, cols-1, CV_8UC3);
	std::vector<node> table(cols*rows);
	std::vector<int> seam(rows);

	calc_energy(image, energy);
	calc_cumulative_energy(energy, table);
	find_seam(table, seam, cols, rows);
	remove_seam(image, seam, tmpout);
	output = tmpout.clone();

	if (direction == 0) {
		transpose(output, output);
		flip(output, output, 1); //transpose+flip(0)=CCW
		//cv::namedWindow("trans", CV_WINDOW_NORMAL);
		//cv::imshow("trans", output);
		//cv::waitKey();
	}
}

int compute_seam_cost(std::vector<node>& table, cv::Size &size_, std::vector<int>& path,int type_)
{
	int cost = 0;
	if (type_ == 0) {
		for (int i = 0; i < size_.height; i++) {
			cost += table[i*size_.width + path[i]].value;
		}
	}
	else {
		for (int i = 0; i < size_.width; i++) {
			cost += table[i*size_.height + path[i]].value;
		}
	}

	return cost;
}



void seamEngine(cv::Mat & image, cv::Mat & output, int reduce_col, int reduce_row)
{
	int count_w=0;
	int count_h = 0;

	cv::Mat input = image.clone();
	cv::Size size_(image.cols, image.rows);

	//two axis
	while (count_h < reduce_row&&count_w < reduce_col) {
		int cost_v = 0;
		int cost_h = 0;
		cv::Mat  energy;
		calc_energy(input, energy);

		//v
		std::vector<node> table_v(size_.width*size_.height);
		std::vector<int> seam_v(size_.height);
		calc_cumulative_energy(energy, table_v);
		find_seam(table_v, seam_v, size_.width, size_.height);
		//cost_v = compute_seam_cost(table_v, size_, seam_v,0);
		for (int i = 0; i < size_.height; i++) {
			cost_v += table_v[i*size_.width + seam_v[i]].value;
		}

		//h
		std::vector<node> table_h(size_.width*size_.height);
		std::vector<int> seam_h(size_.width);
		transpose(energy,energy);
		flip(energy, energy, 1); 
		calc_cumulative_energy(energy, table_h);
		find_seam(table_h, seam_h, size_.height, size_.width);
		//cost_h = compute_seam_cost(table_h, size_, seam_h,1);
		for (int i = 0; i < size_.width; i++) {
			cost_h += table_h[i*size_.height+ seam_h[i]].value;
		}

		if (cost_v < cost_h) {
			cv::Mat tmp(size_.height, size_.width - 1, CV_8UC3);
			remove_seam(input, seam_v, tmp);
			tmp.copyTo(input);
			size_.width-=1;
			count_w+=1;
		}
		else {
			cv::Mat tmp(size_.width, size_.height - 1, CV_8UC3);
			transpose(input, input);
			flip(input, input, 1);
			remove_seam(input, seam_h, tmp);
			transpose(tmp, tmp);
			flip(tmp, tmp, 1);
			tmp.copyTo(input);
			size_.height-=1;
			count_h+=1;
		}
	}

	//水平方向
	while (count_h < reduce_row) {
		cv::Mat  energy;
		cv::Mat tmp(size_.width, size_.height - 1, CV_8UC3);
		transpose(input, input);
		flip(input, input, 1);
		calc_energy(input, energy);

		std::vector<node> table_h(size_.width*size_.height);
		std::vector<int> seam_h(size_.width);
		calc_cumulative_energy(energy, table_h);
		find_seam(table_h, seam_h, size_.height, size_.width);
		remove_seam(input, seam_h, tmp);
		transpose(tmp, tmp);
		flip(tmp, tmp, 1);
		tmp.copyTo(input);
		size_.height -= 1;
		count_h += 1;
	}

	//竖直方向
	while (count_w < reduce_col) {
		cv::Mat  energy;
		cv::Mat tmp(size_.height, size_.width - 1, CV_8UC3);
		calc_energy(input, energy);

		std::vector<node> table_v(size_.width*size_.height);
		std::vector<int> seam_v(size_.height);
		calc_cumulative_energy(energy, table_v);
		find_seam(table_v, seam_v, size_.width, size_.height);
		remove_seam(input, seam_v, tmp);
		tmp.copyTo(input);
		size_.width -= 1;
		count_w += 1;
	}

	output = input;
}

void seamEngine(cv::Mat & image, cv::Mat & output, float scale_w, float scale_h)
{
	int reduce_col,  reduce_row;
	int image_rows = image.rows;
	int image_cols = image.cols;
	reduce_row = image_rows - image_rows*scale_h;
	reduce_col = image_cols - image_cols*scale_w;

	int count_w = 0;
	int count_h = 0;

	cv::Mat input = image.clone();
	cv::Size size_(image.cols, image.rows);

	//two axis
	while (count_h < reduce_row&&count_w < reduce_col) {
		int cost_v = 0;
		int cost_h = 0;
		cv::Mat  energy;


		calc_energy(input, energy);

		//v
		std::vector<node> table_v(size_.width*size_.height);
		std::vector<int> seam_v(size_.height);
		calc_cumulative_energy(energy, table_v);
		find_seam(table_v, seam_v, size_.width, size_.height);
		//cost_v = compute_seam_cost(table_v, size_, seam_v,0);
		for (int i = 0; i < size_.height; i++) {
			cost_v += table_v[i*size_.width + seam_v[i]].value;
		}

		//h
		std::vector<node> table_h(size_.width*size_.height);
		std::vector<int> seam_h(size_.width);
		transpose(energy, energy);
		flip(energy, energy, 1);
		calc_cumulative_energy(energy, table_h);
		find_seam(table_h, seam_h, size_.height, size_.width);
		//cost_h = compute_seam_cost(table_h, size_, seam_h,1);
		for (int i = 0; i < size_.width; i++) {
			cost_h += table_h[i*size_.height + seam_h[i]].value;
		}

		if (cost_v < cost_h) {
			cv::Mat tmp(size_.height, size_.width - 1, CV_8UC3);
			remove_seam(input, seam_v, tmp);
			tmp.copyTo(input);
			size_.width -= 1;
			count_w += 1;
		}
		else {
			cv::Mat tmp(size_.width, size_.height - 1, CV_8UC3);
			transpose(input, input);
			flip(input, input, 1);
			remove_seam(input, seam_h, tmp);
			transpose(tmp, tmp);
			flip(tmp, tmp, 1);
			tmp.copyTo(input);
			size_.height -= 1;
			count_h += 1;
		}
	}

	//水平方向
	while (count_h < reduce_row) {
		cv::Mat  energy;
		cv::Mat tmp(size_.width, size_.height - 1, CV_8UC3);
		transpose(input, input);
		flip(input, input, 1);
		calc_energy(input, energy);

		std::vector<node> table_h(size_.width*size_.height);
		std::vector<int> seam_h(size_.width);
		calc_cumulative_energy(energy, table_h);
		find_seam(table_h, seam_h, size_.height, size_.width);
		remove_seam(input, seam_h, tmp);
		transpose(tmp, tmp);
		flip(tmp, tmp, 1);
		tmp.copyTo(input);
		size_.height -= 1;
		count_h += 1;
	}

	//竖直方向
	while (count_w < reduce_col) {
		cv::Mat  energy;
		cv::Mat tmp(size_.height, size_.width - 1, CV_8UC3);
		calc_energy(input, energy);

		std::vector<node> table_v(size_.width*size_.height);
		std::vector<int> seam_v(size_.height);
		calc_cumulative_energy(energy, table_v);
		find_seam(table_v, seam_v, size_.width, size_.height);
		remove_seam(input, seam_v, tmp);
		tmp.copyTo(input);
		size_.width -= 1;
		count_w += 1;
	}

	output = input;
}

