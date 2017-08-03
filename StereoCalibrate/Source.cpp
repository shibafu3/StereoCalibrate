#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"C:\\opencv\\build\\x86\\vc12\\lib\\opencv_world300d.lib")            // opencv_core
#else
//Releaseモードの場合
#pragma comment(lib,"C:\\opencv\\build\\x86\\vc12\\lib\\opencv_world300.lib") 
#endif
#pragma warning(disable:4996)

#include <stdio.h>
#include <iostream>
#include <vector>
/*OpenCVライブラリ*/
#include <opencv2/opencv.hpp>
//Matとか行列モジュール
#include <opencv2/core/core.hpp>
//GUIモジュール
#include <opencv2/highgui/highgui.hpp>
//キャリブレーションモジュール
#include <opencv2/calib3d/calib3d.hpp>


using namespace std;
using namespace cv;

int main(){
	//入力画像
	vector<Mat> image_l, image_r;
	//入力画像のグレイスケール
	vector<Mat> gray_l, gray_r;
	char image_name_l[100], image_name_r[100];
	int image_num = 1;

	for (int i = 0; i < image_num; i++){
		sprintf(image_name_l, "C:\\Users\\0133752\\Desktop\\l_%02d.jpg", i);
		sprintf(image_name_r, "C:\\Users\\0133752\\Desktop\\r_%02d.jpg", i);

		image_l.push_back(imread(image_name_l));
		image_r.push_back(imread(image_name_r));

		gray_l.push_back(imread(image_name_l, 0));
		gray_r.push_back(imread(image_name_r, 0));
	}

	//画像一枚分のチェスボードのコーナー座標を入れるvector
	vector<Point2f> corners;
	//それを画像の枚数分入れるvector
	vector<vector<Point2f> > image_points_l;
	vector<vector<Point2f> > image_points_r;
	Point chess_size = Point(10, 7);
	for (int i = 0; i < image_l.size(); i++){
		//画像からチェスボードのコーナーを検出する
		findChessboardCorners(image_l[i], Size(10, 7), corners);
		//コーナー座標のサブピクセル精度を求める（入力はグレースケール画像）
		cornerSubPix(gray_l[i], corners, Size(20, 20), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
		//image_pointsにcornersを追加
		image_points_l.push_back(corners);
		drawChessboardCorners(image_l[i], chess_size, corners, true);
		imshow("", image_l[i]);
		waitKey(0);
	}

	for (int i = 0; i < image_r.size(); i++){
		//画像からチェスボードのコーナーを検出する
		findChessboardCorners(image_r[i], Size(10, 7), corners);
		//コーナー座標のサブピクセル精度を求める（入力はグレースケール画像）
		cornerSubPix(gray_r[i], corners, Size(20, 20), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
		//image_pointsにcornersを追加
		image_points_r.push_back(corners);
		cout << corners << endl;
		drawChessboardCorners(image_r[i], chess_size, corners, true);
		imshow("", image_r[i]);
		waitKey(0);
	}

	//画像一枚分のチェスボードの世界座標を入れるvetor
	vector<Point3f> object_c;
	//それを画像の枚数分入れるvector
	vector<vector<Point3f> > object_points;
	for (int y = chess_size.y - 1; y >=0; y--){
		for (int x = chess_size.x - 1; x >= 0; x--){
			object_c.push_back(Point3f(x * 22.5, y * 22.5, 0.0));
		}
	}
	for (int i = 0; i < image_num; i++){
		object_points.push_back(object_c);
	}

	//撮影システムのキャリブレーション（左右のカメラを含めた外部パラメータを求める）
	Mat R, T, E, F;
	//求めたカメラ行列を入れるMat
	Mat camera_matrix_l, camera_matrix_r;
	//求めた歪みベクトルを入れるMat
	Mat dist_coeffs_l, dist_coeffs_r;
	stereoCalibrate(object_points, 
		image_points_l,
		image_points_r,
		camera_matrix_l, dist_coeffs_l, 
		camera_matrix_r, dist_coeffs_r, 
		image_l[0].size(), 
		R, T, E, F);

	//それぞれのカメラの平行化のための回転行列・射影行列およびQを求める
	Mat R_l, R_r, P_l, P_r, Q;
	stereoRectify(camera_matrix_l, dist_coeffs_l,
		camera_matrix_r, dist_coeffs_r, 
		image_l[0].size(), 
		R, T, 
		R_l, R_r, P_l, P_r, Q);

	//平行化と歪み補正を同時に行う"マップ"を求める(左右画像それぞれ)
	Mat mapx_l, mapy_l;
	initUndistortRectifyMap(camera_matrix_l, dist_coeffs_l, R_l, P_l(Rect(0, 0, 3, 3)), image_l[0].size(), CV_32FC1, mapx_l, mapy_l);
	Mat mapx_r, mapy_r;
	initUndistortRectifyMap(camera_matrix_r, dist_coeffs_r, R_r, P_r(Rect(0, 0, 3, 3)), image_r[0].size(), CV_32FC1, mapx_r, mapy_r);


	//xmlファイルに書き出し
	FileStorage cvfsw("C:\\Users\\0133752\\Desktop\\StereoCalibrate.xml", FileStorage::WRITE);
	write(cvfsw, "mapx_l", mapx_l);
	write(cvfsw, "mapy_l", mapy_l);
	write(cvfsw, "mapx_r", mapx_r);
	write(cvfsw, "mapy_r", mapy_r);
	write(cvfsw, "Q", Q);
	cvfsw.release();
	//xmlファイルから読み込み
	FileStorage cvfsr("C:\\Users\\0133752\\Desktop\\StereoCalibrate.xml", FileStorage::READ);
	FileNode node(cvfsr.fs, NULL);
	read(node["mapx_l"], mapx_l);
	read(node["mapy_l"], mapy_l);
	read(node["mapx_r"], mapx_r);
	read(node["mapy_r"], mapy_r);
	read(node["Q"], Q);



	Mat dest_image_l;
	remap(image_l[0], dest_image_l, mapx_l, mapy_l, INTER_LINEAR);
	Mat dest_image_r;
	remap(image_r[0], dest_image_r, mapx_r, mapy_r, INTER_LINEAR);

	imshow("image_l", dest_image_l);
	imshow("image_r", dest_image_r);
	waitKey(0);

	return 0;
}
