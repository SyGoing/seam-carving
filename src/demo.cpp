#include "seam_carving.hpp"
int main() {
	//cv::Mat image = imread("test2.jpg");
	//cv::Mat out;
	//for (int i = 0; i <300; i++) {
	//	seam_carving_single(image, out, 1);
	//	image = out.clone();
	//}
	//cv::imwrite("1_result.jpg", out);
	//cv::imshow("1", out);
	//cv::waitKey();

	cv::Mat image = imread("timg.jpg");
	cv::Mat out;
	seamEngine(image, out, 500,0);
	cv::imwrite("1_result.jpg", out);
	cv::imshow("1", out);
	cv::waitKey();
	return 0;
}