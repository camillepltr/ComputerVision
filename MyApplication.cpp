/*
 * Assignment 2 : Car speed
 * Camille Peltier - peltierc@tcd.ie*
 * Computer Vision - TCD - 2021
 */

#include "Utilities.h"
#include <iostream>
#include <fstream>
#include <list>
#include <experimental/filesystem> // C++-standard header file name
#include <filesystem> // Microsoft-specific implementation header file name
using namespace std::experimental::filesystem::v1;
using namespace std;
using namespace cv;

#define FOCAL_LENGTH_ESTIMATE 1770
#define PLATE_WIDTH_IN_MM 465
#define PLATE_HEIGHT_IN_MM 100
#define FRAMES_PER_SECOND 29.97
#define REQUIRED_DICE 0.8
#define RESOLUTION_WIDTH 960
#define RESOLUTION_HEIGHT 540
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
const int NUMBER_OF_FRAMES_FOR_DISTANCES = sizeof(FRAMES_FOR_DISTANCES) / (sizeof(FRAMES_FOR_DISTANCES[0]));

// Some manually set settings
const int MAX_DISTANCE_IN_PIXELS_BETWEEN_FRAMES = 150;
const bool DISPLAY_STEPS_RESULT = false; // True to make screenshots for the report

Mat structuring_element_3x3(3, 3, CV_8U, Scalar(1));
Mat structuring_element_5x5(5, 5, CV_8U, Scalar(1));

// Template features
double plate_aspect_ratio = (double)PLATE_HEIGHT_IN_MM / (double)PLATE_WIDTH_IN_MM;
double plate_rectangularity = 1.0;
double plate_angle = 90.0;

//For CCA
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;

// To store results
RotatedRect located_plates[NUMBER_OF_PLATES];
float computed_distances[NUMBER_OF_FRAMES_FOR_DISTANCES - 1];
float computed_speeds[NUMBER_OF_FRAMES_FOR_DISTANCES - 1];
int last_located_plate_index = 0;


// To display intermediate treatment results, for screenshots, will dispolay the current frame if it has the frame_to_display frame number
void displayFrame(Mat image, const char* description, int current_frame) {
	int frame_to_display = 155; // Arbitrary
	if (current_frame == frame_to_display) {
		char frame_no[20];
		sprintf(frame_no, "%d", current_frame);
		Point frame_no_location(5, 15);
		Scalar frame_no_colour(0x00, 0x00, 0xFF);

		putText(image, frame_no, frame_no_location, FONT_HERSHEY_SIMPLEX, 0.4, frame_no_colour);
		imshow(description, image);
	}
}

/******************* Methods for the evaluation of the plate location *******************/
double computeDiceCoefficient(int index) {
	Rect located_plate = located_plates[index].boundingRect();

	// Ground truth rectangle
	Point2f truth_top_left_corner = Point2f(LICENCE_PLATE_LOCATIONS[index][1], LICENCE_PLATE_LOCATIONS[index][2]);
	Size2f truth_size = Size2f(LICENCE_PLATE_LOCATIONS[index][3], LICENCE_PLATE_LOCATIONS[index][4]);
	Rect truth = Rect(truth_top_left_corner, truth_size);

	// Intersection
	double plates_intersection = (located_plate & truth).area();

	//Dice coefficient
	return 2 * plates_intersection / ((double)truth.area() + (double)located_plate.area());
}

void evaluatePlateLocation() {
	int true_positives = 0;
	int false_positives = 0;
	int false_negatives = 0;

	for (int i = 0; i < NUMBER_OF_PLATES; i++) {
		if (located_plates[i].center == Point2f(0.0, 0.0)) {
			false_negatives++; // No plate found
		}
		else {
			double dice = computeDiceCoefficient(i);
			if (dice > REQUIRED_DICE) {
				true_positives++;
			}
			else {
				false_positives++;
			}
		}
	}

	double precision = ((double)true_positives) / ((double)(true_positives + false_positives));
	double recall = ((double)true_positives) / ((double)(true_positives + false_negatives));
	cout << endl << "*************************** Evaulation for the location of the plate ****************************" << endl;
	cout << "Counts of TP : " << true_positives << endl;
	cout << "Counts of FP : " << false_positives << endl;
	cout << "Counts of FN : " << false_negatives << endl;
	cout << "Metrics : " << endl;
	cout << " - Recall : " << recall << endl;
	cout << " - Precision : " << precision << endl;
	cout << "**************************************************************************************************" << endl;
}

/******************* Methods for the computation and evaluation of distances/speeds *******************/
RotatedRect estimateMissingPlateLocation(int index) {
	// Find the last located plate
	int prev = index - 1;
	RotatedRect previous_rect;
	while (prev > 0 && located_plates[prev].center == Point2f(0, 0)) {
		prev--;
	}
	previous_rect = located_plates[prev];

	// Find the next located plate
	int next = index + 1;
	RotatedRect next_rect;
	while (prev < NUMBER_OF_PLATES && located_plates[next].center == Point2f(0, 0)) {
		next++;
	}
	next_rect = located_plates[next];

	if (prev != 0 && next != NUMBER_OF_PLATES) {
		RotatedRect res;
		int delta = next - prev;
		res.center.x = previous_rect.center.x + (next_rect.center.x - previous_rect.center.x) / delta;
		res.center.y = previous_rect.center.y + (next_rect.center.y - previous_rect.center.y) / delta;
		res.size.width = previous_rect.size.width + (next_rect.size.width - previous_rect.size.width) / delta;
		res.size.height = previous_rect.size.height + (next_rect.size.height - previous_rect.size.height) / delta;
		return res;
	}
	else if (prev == 0 && next != NUMBER_OF_PLATES) {
		return next_rect;
	}
	else if (prev != 0 && next == NUMBER_OF_PLATES) {
		return next_rect;
	} 
	else {
		return RotatedRect();
	}
}

void computeDistancesAndSpeeds() {
	cout << endl << "*************************** Distances and speeds between frames ****************************" << endl;
	for (int i = 1; i < NUMBER_OF_FRAMES_FOR_DISTANCES; i++) {
		int previous_frame_index = FRAMES_FOR_DISTANCES[i-1] - 1; //In the located_plates array, frame number i is at index i-1
		int frame_index = FRAMES_FOR_DISTANCES[i] - 1;
		
		if (located_plates[frame_index].center == Point2f(0, 0)) {
			cout << endl << "Missing plate location for frame " << frame_index + 1 << ", location estimated." << endl;
			located_plates[frame_index] = estimateMissingPlateLocation(frame_index);
		}
		
		// Distance 
		float ci = RESOLUTION_WIDTH / 2.0;
		float dCamera1 = sqrt(pow(FOCAL_LENGTH_ESTIMATE, 2) + pow(ci - located_plates[previous_frame_index].center.x, 2));
		float dCamera2 = sqrt(pow(FOCAL_LENGTH_ESTIMATE, 2) + pow(ci - located_plates[frame_index].center.x, 2));
		float d1 = dCamera1 * (float)PLATE_WIDTH_IN_MM / located_plates[previous_frame_index].size.height;
		float d2 = dCamera2 * (float)PLATE_WIDTH_IN_MM / located_plates[frame_index].size.height;
		float cos_theta = (pow(FOCAL_LENGTH_ESTIMATE, 2) + (ci - located_plates[previous_frame_index].center.x) * (ci - located_plates[frame_index].center.x)) / (dCamera1 * dCamera2);
		float distance = sqrt(pow(d1,2) + pow(d2,2) - 2*d1*d2*cos_theta);
		
		cout << endl << "Between frames  " << previous_frame_index + 1 << " and " << frame_index + 1 << endl;
		cout << "  Distance : " << distance << " mm" << endl;
		computed_distances[i-1] = distance;

		// Time
		double delta_t = (double)(frame_index - previous_frame_index) / FRAMES_PER_SECOND;
		cout << "  Delta t : " << delta_t << " seconds " << endl;
		
		// Speed in kmph
		delta_t /= 3600.0;
		distance /= 1000000.0;

		float speed = distance / delta_t;
		cout << "  -> Speed  : " << speed << " Km/H" << endl;
		computed_speeds[i-1] = speed;
	}
}

void evaluateDistancesAndSpeeds() {
	float mae_distances = 0.0f;
	for (int i = 0; i < NUMBER_OF_FRAMES_FOR_DISTANCES - 1; i++) {
		mae_distances += abs(computed_distances[i] - DISTANCES_TRAVELLED_IN_MM[i]);
	}
	mae_distances /= (NUMBER_OF_FRAMES_FOR_DISTANCES -1);

	float mae_speeds = 0.0f;
	for (int i = 0; i < NUMBER_OF_FRAMES_FOR_DISTANCES - 1; i++) {
		mae_speeds += abs(computed_speeds[i] - SPEEDS_IN_KMPH[i]);
	}
	mae_speeds /= (NUMBER_OF_FRAMES_FOR_DISTANCES - 1);

	cout << endl << "********************* Evaulation of the computed distances and speeds between 9 frames *******************" << endl;
	cout << "Mean Average Error for travelled distances : " << mae_distances << endl;
	cout << "Mean Average Error for speeds : " << mae_speeds << endl;
	cout << "**************************************************************************************************" << endl;

}

/******************* Methods for the location of the plate *******************/
void drawLocatedPlate(int &frame_number, Mat &current_frame, RotatedRect &min_bounding_rectangle, int &closest_contour_index, Mat &contours_image, vector<vector<Point>> &contours, vector<Vec4i> &hierarchy) {

	// Color region on current frame and contour images
	Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
	drawContours(contours_image, contours, closest_contour_index, colour, cv::FILLED, 8, hierarchy);
	drawContours(current_frame, contours, closest_contour_index, colour, cv::FILLED, 8, hierarchy);

	// Draw the minimum bounding rectangle
	Point2f bounding_rect_points[4];
	min_bounding_rectangle.points(bounding_rect_points);
	line(current_frame, bounding_rect_points[0], bounding_rect_points[1], Scalar(0, 0, 127));
	line(current_frame, bounding_rect_points[1], bounding_rect_points[2], Scalar(0, 0, 127));
	line(current_frame, bounding_rect_points[2], bounding_rect_points[3], Scalar(0, 0, 127));
	line(current_frame, bounding_rect_points[3], bounding_rect_points[0], Scalar(0, 0, 127));

	// Display features
	char output[500];
	double area = contourArea(contours[closest_contour_index]) + contours[closest_contour_index].size() / 2 + 1; //Optimize to avoid recalculation ?
	double aspect_ratio = min_bounding_rectangle.size.aspectRatio();
	double rectangularity = area / ((double)min_bounding_rectangle.size.width * (double)min_bounding_rectangle.size.height);
	double angle = min_bounding_rectangle.angle;
	if (DISPLAY_STEPS_RESULT) { displayFrame(contours_image, "Located plate", frame_number); }

}

void findBestMatch(int &frame_number, Mat &current_frame, Mat &contours_image, vector<double> &matching_scores, vector<RotatedRect> &min_bounding_rectangles) {
	// Find contour with best matching score with the template plate + Can't have moved very far from previous frame
	int best_contour_index = min_element(matching_scores.begin(), matching_scores.end()) - matching_scores.begin();

	// If it's the first frame or the plate hasn't moved to far : Accept it
	if (frame_number == 1 ||
		(DistanceBetweenPoints((Point2i)located_plates[last_located_plate_index].center, (Point2i)min_bounding_rectangles[best_contour_index].center) < MAX_DISTANCE_IN_PIXELS_BETWEEN_FRAMES)) {
		located_plates[frame_number - 1] = min_bounding_rectangles[best_contour_index];
		last_located_plate_index = frame_number - 1;
		drawLocatedPlate(frame_number, current_frame, min_bounding_rectangles[best_contour_index], best_contour_index, contours_image, contours, hierarchy);
	}
	else {
		// Find second best
		matching_scores[best_contour_index] = DBL_MAX;
		best_contour_index = min_element(matching_scores.begin(), matching_scores.end()) - matching_scores.begin();
		if (frame_number == 1 ||
			(DistanceBetweenPoints((Point2i)located_plates[last_located_plate_index].center, (Point2i)min_bounding_rectangles[best_contour_index].center) < MAX_DISTANCE_IN_PIXELS_BETWEEN_FRAMES)) {
			located_plates[frame_number - 1] = min_bounding_rectangles[best_contour_index];
			last_located_plate_index = frame_number - 1;
			drawLocatedPlate(frame_number, current_frame, min_bounding_rectangles[best_contour_index], best_contour_index, contours_image, contours, hierarchy);
		}

	}
}

void findPlate(int& frame_number, Mat& current_frame, int & last_located_plate_index, Mat & static_background_image) {
	if (frame_number <= NUMBER_OF_PLATES) { //Only work on the frames where we have ground truth to exploit results

		//Static background
		Mat difference_image, thresholded_difference_im;
		absdiff(current_frame, static_background_image, difference_image);
		cvtColor(difference_image, thresholded_difference_im, COLOR_BGR2GRAY);
		threshold(thresholded_difference_im, thresholded_difference_im, 20, 255, THRESH_BINARY);
		if (DISPLAY_STEPS_RESULT) { displayFrame(thresholded_difference_im, "Binary moving object pixels", frame_number); }

		//Add closing and opening treatment on binary
		Mat opened_image, dilated_image, moving_object_pixels;
		morphologyEx(thresholded_difference_im, opened_image, MORPH_OPEN, structuring_element_3x3);
		dilate(opened_image, dilated_image, structuring_element_5x5);
		if (DISPLAY_STEPS_RESULT) { displayFrame(dilated_image, "Binary moving object pixels after geometry", frame_number); }

		current_frame.copyTo(moving_object_pixels, dilated_image);
		if (DISPLAY_STEPS_RESULT) { displayFrame(moving_object_pixels, "Original moving object pixels after geometry", frame_number); }

		//OTSU thresholding on the original grayscale image for the moving object pixels only, to select bright areas of the moving object
		Mat gray_moving_pixels, otsu_image;
		cvtColor(moving_object_pixels, gray_moving_pixels, COLOR_BGR2GRAY);
		threshold(gray_moving_pixels, otsu_image, 0, 255, THRESH_BINARY | THRESH_OTSU);
		if (DISPLAY_STEPS_RESULT) { displayFrame(otsu_image, "OTSU result", frame_number); }

		//CCA to obtain regions
		findContours(otsu_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
		Mat contours_image = Mat::zeros(otsu_image.size(), CV_8UC3);
		Mat contours_on_original;
		current_frame.copyTo(contours_on_original);

		if (DISPLAY_STEPS_RESULT) {
			for (int contour_number = 0; (contour_number < (int)contours.size()); contour_number++)
			{
				Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
				drawContours(contours_image, contours, contour_number, colour, cv::FILLED, 8, hierarchy);
				drawContours(contours_on_original, contours, contour_number, colour, cv::FILLED, 8, hierarchy);
			}
			displayFrame(contours_image, "CCA result", frame_number);
			displayFrame(contours_on_original, "CCA result on original image", frame_number);
		}

		//SPR
		vector<double> matching_scores(contours.size());
		vector<RotatedRect> min_bounding_rectangles(contours.size());
		for (int contour_number = 0; (contour_number < (int)contours.size()); contour_number++)
		{
			if (contours[contour_number].size() > 60)
			{
				min_bounding_rectangles[contour_number] = minAreaRect(contours[contour_number]);
				double area = contourArea(contours[contour_number]) + contours[contour_number].size() / 2 + 1;
				double aspect_ratio = min_bounding_rectangles[contour_number].size.aspectRatio();
				double rectangularity = area / ((double)min_bounding_rectangles[contour_number].size.width * (double)min_bounding_rectangles[contour_number].size.height);
				double angle = min_bounding_rectangles[contour_number].angle;
				if (rectangularity < 0.7 || rectangularity > 1.29 || aspect_ratio < 0.1 || aspect_ratio > 0.3) { // Eliminate region
					matching_scores[contour_number] = DBL_MAX;
				}
				else { 	// Matching score : Distance to the plate based on aspect ratio rectangularity and angle features
					matching_scores[contour_number] = sqrt(pow(plate_aspect_ratio - aspect_ratio, 2) + pow(plate_rectangularity - rectangularity, 2) + pow(plate_angle - angle, 2));
				}
			}
			else {
				matching_scores[contour_number] = DBL_MAX;
			}
		}
		findBestMatch(frame_number, current_frame, contours_image, matching_scores, min_bounding_rectangles);

		// Display (with located plate and frame number)
		char frame_no[20];
		sprintf(frame_no, "%d", frame_number);
		Point frame_no_location(5, 15);
		Scalar frame_no_colour(0, 0, 0xFF);
		putText(current_frame, frame_no, frame_no_location, FONT_HERSHEY_SIMPLEX, 0.4, frame_no_colour);
		if (true) { displayFrame(current_frame, "Located plate", frame_number); }

		imshow("Video", current_frame);
	}
}

/*******************************************************************************/
int main(int argc, const char** argv)
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
		Mat current_frame;
		video.set(cv::CAP_PROP_POS_FRAMES, 1);
		video >> current_frame;
		int frame_number = 1;
		double last_time = static_cast<double>(getTickCount());
		double frame_rate = video.get(cv::CAP_PROP_FPS);
		double time_between_frames = 1000.0 / frame_rate;

		while (!current_frame.empty())
		{
			// Locate the plate
			findPlate(frame_number, current_frame, last_located_plate_index, static_background_image);

			// Go to next frame
			double current_time = static_cast<double>(getTickCount());
			double duration = (current_time - last_time) / getTickFrequency() / 1000.0;
			int delay = (time_between_frames > duration) ? ((int)(time_between_frames - duration)) : 1;
			last_time = current_time;
			char c = cv::waitKey(delay);
			for (int i = 0; i < 1; i++)
				video >> current_frame;
			frame_number++;

		}

		// Evaluation for plate location
		evaluatePlateLocation();

		// Distances and speed
		computeDistancesAndSpeeds();
		evaluateDistancesAndSpeeds();
		cv::destroyAllWindows();
	}
}
