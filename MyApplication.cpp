#include "Utilities.h"
#include <iostream>
#include <fstream>
#include <list>
#include <experimental/filesystem> // C++-standard header file name
#include <filesystem> // Microsoft-specific implementation header file name
using namespace std::experimental::filesystem::v1;
using namespace std;

#define FOCAL_LENGTH_ESTIMATE 1770
#define PLATE_WIDTH_IN_MM 465
#define PLATE_HEIGHT_IN_MM 100
#define FRAMES_PER_SECOND 29.97
#define REQUIRED_DICE 0.8
const int LICENCE_PLATE_LOCATIONS[][5] = { {1, 67, 88, 26, 6}, {2, 67, 88, 26, 6}, {3, 68, 88, 26, 6},
	{4, 69, 88, 26, 6}, {5, 70, 89, 26, 6}, {6, 70, 89, 27, 6}, {7, 71, 89, 27, 6}, {8, 73, 89, 27, 6},
	{9, 73, 90, 27, 6}, {10, 74, 90, 27, 6}, {11, 75, 90, 27, 6}, {12, 76, 90, 27, 6}, {13, 77, 91, 27, 6},
	{14, 78, 91, 27, 6}, {15, 78, 91, 27, 6}, {16, 79, 91, 27, 6}, {17, 80, 92, 27, 6}, {18, 81, 92, 27, 6},
	{19, 81, 92, 28, 6}, {20, 82, 93, 28, 6}, {21, 83, 93, 28, 6}, {22, 83, 93, 28, 6}, {23, 84, 93, 28, 6},
	{24, 85, 94, 28, 6}, {25, 85, 94, 28, 6}, {26, 86, 94, 28, 6}, {27, 86, 94, 28, 6}, {28, 86, 95, 29, 6},
	{29, 87, 95, 29, 6}, {30, 87, 95, 29, 6}, {31, 88, 95, 29, 6}, {32, 88, 96, 29, 6}, {33, 89, 96, 29, 6},
	{34, 89, 96, 29, 6}, {35, 89, 97, 29, 6}, {36, 90, 97, 29, 6}, {37, 90, 97, 30, 6}, {38, 91, 98, 30, 6},
	{39, 91, 98, 30, 6}, {40, 92, 98, 30, 7}, {41, 92, 99, 30, 7}, {42, 93, 99, 30, 7}, {43, 93, 99, 30, 7},
	{44, 94, 100, 30, 7}, {45, 95, 100, 30, 7}, {46, 95, 101, 30, 7}, {47, 96, 101, 30, 7}, {48, 97, 102, 30, 7},
	{49, 97, 102, 31, 7}, {50, 98, 102, 31, 7}, {51, 99, 103, 31, 7}, {52, 99, 103, 32, 7}, {53, 100, 104, 32, 7},
	{54, 101, 104, 32, 7}, {55, 102, 105, 32, 7}, {56, 103, 105, 32, 7}, {57, 104, 106, 32, 7}, {58, 105, 106, 32, 7},
	{59, 106, 107, 32, 7}, {60, 107, 107, 32, 7}, {61, 108, 108, 32, 7}, {62, 109, 108, 33, 7}, {63, 110, 109, 33, 7},
	{64, 111, 109, 33, 7}, {65, 112, 110, 34, 7}, {66, 113, 111, 34, 7}, {67, 114, 111, 34, 7}, {68, 116, 112, 34, 7},
	{69, 117, 112, 34, 8}, {70, 118, 113, 35, 8}, {71, 119, 113, 35, 8}, {72, 121, 114, 35, 8}, {73, 122, 114, 35, 8},
	{74, 124, 115, 35, 8}, {75, 125, 116, 36, 8}, {76, 127, 116, 36, 8}, {77, 128, 117, 36, 8}, {78, 130, 118, 36, 8},
	{79, 132, 118, 36, 9}, {80, 133, 119, 37, 9}, {81, 135, 120, 37, 9}, {82, 137, 121, 37, 9}, {83, 138, 122, 38, 9},
	{84, 140, 122, 38, 9}, {85, 142, 123, 38, 9}, {86, 144, 124, 38, 9}, {87, 146, 125, 38, 9}, {88, 148, 126, 39, 9},
	{89, 150, 127, 39, 9}, {90, 152, 128, 39, 9}, {91, 154, 129, 40, 9}, {92, 156, 129, 40, 10}, {93, 158, 130, 40, 10},
	{94, 160, 131, 41, 10}, {95, 163, 133, 41, 10}, {96, 165, 133, 41, 10}, {97, 167, 135, 42, 10}, {98, 170, 135, 42, 10},
	{99, 172, 137, 43, 10}, {100, 175, 138, 43, 10}, {101, 178, 139, 43, 10}, {102, 180, 140, 44, 10}, {103, 183, 141, 44, 10},
	{104, 186, 142, 44, 11}, {105, 188, 143, 45, 11}, {106, 192, 145, 45, 11}, {107, 195, 146, 45, 11}, {108, 198, 147, 45, 11},
	{109, 201, 149, 46, 11}, {110, 204, 150, 47, 11}, {111, 207, 151, 47, 11}, {112, 211, 152, 47, 11}, {113, 214, 154, 48, 11},
	{114, 218, 155, 48, 12}, {115, 221, 157, 49, 12}, {116, 225, 158, 50, 12}, {117, 229, 160, 50, 12}, {118, 234, 162, 50, 12},
	{119, 237, 163, 51, 12}, {120, 241, 164, 52, 12}, {121, 245, 166, 52, 12}, {122, 250, 168, 52, 12}, {123, 254, 169, 53, 12},
	{124, 258, 171, 54, 12}, {125, 263, 173, 55, 12}, {126, 268, 175, 55, 12}, {127, 273, 177, 55, 12}, {128, 278, 179, 56, 13},
	{129, 283, 181, 57, 13}, {130, 288, 183, 57, 13}, {131, 294, 185, 58, 13}, {132, 299, 187, 59, 13}, {133, 305, 190, 59, 13},
	{134, 311, 192, 60, 13}, {135, 317, 194, 60, 14}, {136, 324, 196, 60, 14}, {137, 330, 198, 61, 14}, {138, 336, 201, 63, 14},
	{139, 342, 203, 64, 14}, {140, 349, 206, 65, 14}, {141, 357, 208, 65, 15}, {142, 364, 211, 66, 15}, {143, 372, 214, 67, 15},
	{144, 379, 217, 68, 15}, {145, 387, 220, 69, 15}, {146, 396, 223, 70, 15}, {147, 404, 226, 71, 16}, {148, 412, 229, 72, 16},
	{149, 422, 232, 73, 17}, {150, 432, 236, 74, 17}, {151, 440, 239, 75, 18}, {152, 450, 243, 76, 18}, {153, 460, 247, 77, 18},
	{154, 470, 250, 78, 19}, {155, 482, 254, 78, 19}, {156, 492, 259, 81, 19}, {157, 504, 263, 82, 20}, {158, 516, 268, 83, 20},
	{159, 528, 272, 85, 21}, {160, 542, 277, 85, 21}, {161, 554, 282, 88, 21}, {162, 569, 287, 88, 22}, {163, 584, 292, 89, 22},
	{164, 598, 297, 91, 23}, {165, 614, 302, 92, 24}, {166, 630, 308, 94, 24}, {167, 646, 314, 96, 25}, {168, 664, 320, 97, 26},
	{169, 681, 327, 100, 26}, {170, 700, 334, 101, 27}, {171, 719, 341, 103, 28}, {172, 740, 349, 105, 29}, {173, 762, 357, 107, 29},
	{174, 784, 365, 109, 30}, { 175, 808, 374, 110, 31 }, { 176, 832, 383, 113, 32 } };
const int NUMBER_OF_PLATES = sizeof(LICENCE_PLATE_LOCATIONS) / (sizeof(LICENCE_PLATE_LOCATIONS[0]));
const int FRAMES_FOR_DISTANCES[] = { 54,   70,   86,  101,  115,  129,  143,  158,  172 };
const int DISTANCES_TRAVELLED_IN_MM[] = { 2380, 2380, 2400, 2380, 2395, 2380, 2385, 2380 };
const double SPEEDS_IN_KMPH[] = { 16.0, 16.0, 17.3, 18.3, 18.5, 18.3, 17.2, 18.3 };

void MyApplication()
{
	string video_filename("Media/CarSpeedTest1.mp4");
	VideoCapture video;
	video.open(video_filename);

	string filename("Media/LicencePlateTemplate.png");
	Mat template_image = imread(filename, -1);
	string background_filename("Media/CarSpeedTest1EmptyFrame.jpg");
	Mat static_background_image = imread(background_filename, -1);
	if ((!video.isOpened()) || (template_image.empty()) || (static_background_image.empty()))
	{
		if (!video.isOpened())
			cout << "Cannot open video file: " << video_filename << endl;
		if (template_image.empty())
			cout << "Cannot open image file: " << filename << endl;
		if (static_background_image.empty())
			cout << "Cannot open image file: " << background_filename << endl;
	}
	else
	{
		Mat gray_template_image;
		cvtColor(template_image, gray_template_image, COLOR_BGR2GRAY);

		Mat current_frame;
		video.set(cv::CAP_PROP_POS_FRAMES, 1);
		video >> current_frame;
		int frame_number = 1;
		double last_time = static_cast<double>(getTickCount());
		double frame_rate = video.get(cv::CAP_PROP_FPS);
		double time_between_frames = 1000.0 / frame_rate;
		while (!current_frame.empty())
		{
			char frame_no[20];
			sprintf(frame_no, "%d", frame_number);
			Point frame_no_location(5, 15);
			Scalar frame_no_colour(0, 0, 0xFF);
			putText(current_frame, frame_no, frame_no_location, FONT_HERSHEY_SIMPLEX, 0.4, frame_no_colour);
			cv::imshow("Video", current_frame);
			double current_time = static_cast<double>(getTickCount());
			double duration = (current_time - last_time) / getTickFrequency() / 1000.0;
			int delay = (time_between_frames > duration) ? ((int)(time_between_frames - duration)) : 1;
			last_time = current_time;
			char c = cv::waitKey(delay);
			for (int i = 0; i < 1; i++)
				video >> current_frame;
			frame_number++;
		}
		cv::destroyAllWindows();
	}
}

