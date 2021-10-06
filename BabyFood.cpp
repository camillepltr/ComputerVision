// Lab 1 : Baby food
// Camille Peltier - TCD - Computer Vision - 2021

#include "Utilities.h"

float getAverage(Mat& image)
{
	float sum = 0.0;
	for (int row = 0; row < image.rows; row++) {
		for (int col = 0; col < image.cols; col++) {
			sum += image.at<uchar>(row, col);
		}
	}
	return sum / image.total();
}

int BabyFood()
{
	// Load images
	char* file_location = "Media/BabyFood/";
	char* image_files[] = {
		"BabyFood-Sample0.JPG", //Samples
		"BabyFood-Sample1.JPG",
		"BabyFood-Sample2.JPG",
		"BabyFood-Test1.JPG",  //Test 1
		"BabyFood-Test2.JPG",
		"BabyFood-Test3.JPG",
		"BabyFood-Test4.JPG",
		"BabyFood-Test5.JPG", //5
		"BabyFood-Test6.JPG",
		"BabyFood-Test7.JPG",
		"BabyFood-Test8.JPG",
		"BabyFood-Test9.JPG",
		"BabyFood-Test10.JPG", //10
		"BabyFood-Test12.JPG",
		"BabyFood-Test12.JPG",
		"BabyFood-Test13.JPG",
		"BabyFood-Test14.JPG",
		"BabyFood-Test15.JPG", //15
		"BabyFood-Test16.JPG",
		"BabyFood-Test17.JPG" ,
		"BabyFood-Test18.JPG"
	};
	int number_of_images = sizeof(image_files) / sizeof(image_files[0]);
	Mat* image = new Mat[number_of_images];
	for (int file_no = 0; (file_no < number_of_images); file_no++)
	{
		string filename(file_location);
		filename.append(image_files[file_no]);
		image[file_no] = imread(filename, -1);
		if (image[file_no].empty())
		{
			cout << "Could not open " << image[file_no] << endl;
			return -1;
		}
	}

	// In this method we will simply use the average brigthness on the greyscale image
	// Convert to greyscale and compute average brightness
	Mat* image_gray = new Mat[number_of_images];
	float* brigthness_avg = new float[number_of_images];
	for (int i = 0; (i < number_of_images); i++)
	{
		cvtColor(image[i], image_gray[i], COLOR_BGR2GRAY);
		brigthness_avg[i] = getAverage(image_gray[i]);
	}

	//Display samples
	Mat* small_samples_display = new Mat[3];
	resize(image[0], small_samples_display[0], Size(image[0].cols / 2, image[0].rows / 2));
	resize(image[1], small_samples_display[1], Size(image[1].cols / 2, image[1].rows / 2));
	resize(image[2], small_samples_display[2], Size(image[2].cols / 2, image[2].rows / 2));
	Mat output1 = JoinImagesHorizontally(small_samples_display[0], "Sample 0 : no spoon - WRONG", small_samples_display[1], "Sample 1 : 1 spoon - OK", 4);
	Mat output2 = JoinImagesHorizontally(output1, "", small_samples_display[2], "Sample 2 : 2 spoons - WRONG", 4);
	imshow("Samples", output2);
	char c = cv::waitKey();
	cv::destroyAllWindows();
	Mat* small_samples_gray_display = new Mat[3];
	resize(image_gray[0], small_samples_gray_display[0], Size(image_gray[0].cols / 2, image_gray[0].rows / 2));
	resize(image_gray[1], small_samples_gray_display[1], Size(image_gray[1].cols / 2, image_gray[1].rows / 2));
	resize(image_gray[2], small_samples_gray_display[2], Size(image_gray[2].cols / 2, image_gray[2].rows / 2));
	Mat output3 = JoinImagesHorizontally(small_samples_gray_display[0], "Sample 0 greyscale - Average brightness : " + to_string(brigthness_avg[0]), small_samples_gray_display[1], "Sample 1 greyscale- Average brightness : " + to_string(brigthness_avg[1]), 4);
	Mat output4 = JoinImagesHorizontally(output3, "", small_samples_gray_display[2], "Sample 2 greyscale- Average brightness : " + to_string(brigthness_avg[2]), 4);
	Mat output5 = JoinImagesVertically(output2, "", output4, "", 4);
	imshow("Samples", output4);
	c = cv::waitKey();
	cv::destroyAllWindows();

	//Display test images and results 
	for (int i = 3; (i < number_of_images); i++)
	{
		string text = image_files[i];
		text += " - Average brightness : " + to_string(brigthness_avg[i]);
		if ((abs(brigthness_avg[i] - brigthness_avg[1]) < abs(brigthness_avg[i] - brigthness_avg[0])) && (abs(brigthness_avg[i] - brigthness_avg[1]) < abs(brigthness_avg[i] - brigthness_avg[2]))) {
			text += " - OK";
		}
		else if (abs(brigthness_avg[i] - brigthness_avg[0]) < abs(brigthness_avg[i] - brigthness_avg[2])) {
			text += " - WRONG (0 spoon)";
		}
		else {
			text += " - WRONG (2 spoons or more)";
		}
		imshow(text, image_gray[i]);
		c = cv::waitKey();
		cv::destroyAllWindows();
	}
	return 0;
}