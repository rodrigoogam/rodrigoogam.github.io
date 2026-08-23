#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <chrono>
#include <iostream>
#include <fstream>

#include "shared_data.hpp"
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// for print error message
#include <string.h>
#include <errno.h>



using namespace cv;
using namespace std;
using namespace std::chrono;

#define NAMED_PIPE "/var/lock/pipename"
double coord_1 = 0;
double coord_2 = 0;

double prev1 = 0;
double prev2 = 0;

vector<double> xc;
vector<double> yc;
vector<double> xu;
vector<double> yu;
vector<double> dt;

double fillx = 0.5;
double filly = 0.4;

int counts = 0;

void plot() {
    ofstream myfile("../dataM.csv");
    myfile << "x[n]"<<","<<"y[n]"<<","<<"dt[n]"<<endl;
    int vsize = xc.size();
    for (int n = 0; n<vsize;n++){
        myfile<<xc[n]<<","<<yc[n]<<","<<dt[n]<<endl;
    }

    myfile.close();
    //system("python3 ./plots.py");
    //system("python3 ../plots2.py");
}

double filter(double un, double fill){
    double salida = (10*un-10*fill)*0.003333+fill;
    return salida; 
}

int main(int argc, char * argv[]) try{
    //sharedMem
    const char* name = "/my_shared_mem";
    int fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    ftruncate(fd, sizeof(SharedData));
    auto* shared = static_cast<SharedData*>(mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    close(fd);

    shared->data_ready = false;


    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map1;
    // Config for streams
    rs2::config* config_1 = new rs2::config();
    // Request a specific configuration
    config_1->enable_device("034422072062");
    config_1->enable_stream(RS2_STREAM_DEPTH, 0, 848, 100, RS2_FORMAT_Z16, 300);
    config_1->enable_stream(RS2_STREAM_INFRARED, 1, 848, 100, RS2_FORMAT_Y8, 300);
    rs2::pipeline pipe_1;
    rs2::pipeline_profile selection1 = pipe_1.start(*config_1);
    rs2::decimation_filter dec_filter1;  // Decimation - reduces depth frame density
    rs2::temporal_filter temp_filter1;   // Temporal   - reduces temporal noise
    rs2::hole_filling_filter hole_filter1;
    dec_filter1.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
    temp_filter1.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4);
    temp_filter1.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,20);
    hole_filter1.set_option(RS2_OPTION_HOLES_FILL,1);

    rs2::device selected_device1 = selection1.get_device();
    auto depth_sensor1 = selected_device1.first<rs2::depth_sensor>();

    if (depth_sensor1.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor1.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        //depth_sensor1.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter

    }
    depth_sensor1.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.f);
    depth_sensor1.set_option(RS2_OPTION_EXPOSURE,1000);
    SimpleBlobDetector::Params params; 
    // Change thresholds
    params.minThreshold = 200;
    params.maxThreshold = 255;
    
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 2000;
    
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.5;
    
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.7;
    
    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;


    //const auto window_name1 = "Display Image1";
    //namedWindow(window_name1, WINDOW_AUTOSIZE);

    const auto window_name3 = "Display Image3";
    namedWindow(window_name3, WINDOW_AUTOSIZE);


    while ((waitKey(1)&0xFF) !='q' && getWindowProperty(window_name3, WND_PROP_AUTOSIZE) >= 0)
    {   
        auto start = high_resolution_clock::now();
        rs2::frameset data1 = pipe_1.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth1 = data1.get_depth_frame().apply_filter(color_map1);
        rs2::frame ir1 = data1.get_infrared_frame();
        rs2::frame filtered1 = depth1;
       // Note the concatenation of output/input frame to build up a chain
       filtered1 = dec_filter1.process(depth1);
       filtered1 = temp_filter1.process(filtered1);
       filtered1 = hole_filter1.process(filtered1);

        // Query frame size (width and height)
        const int w1 = filtered1.as<rs2::video_frame>().get_width();
        const int h1 = filtered1.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image1(Size(w1, h1), CV_8UC3, (void*)filtered1.get_data(), Mat::AUTO_STEP);
         // Query frame size (width and height)
        const int w3 = ir1.as<rs2::video_frame>().get_width();
        const int h3 = ir1.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image3_ir(Size(w3, h3), CV_8UC1, (void*)ir1.get_data(), Mat::AUTO_STEP);
        Mat image3;
        threshold(image3_ir, image3, 100, 255, THRESH_BINARY_INV);//converting grayscale image stored in converted matrix into binary image//
        Mat cropped_image = image3(Range(), Range(165,650));
        
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        // Detect blobs.
        std::vector<KeyPoint> keypoints;
        detector->detect(image3, keypoints);
        
        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        Mat image3_blob;
        drawKeypoints( image3, keypoints, image3_blob, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
   
        // Update the window with new data
        //imshow(window_name1, image1);
        imshow(window_name3, image3_blob);

        if(keypoints.size()>0){
            prev1 =coord_1;
            prev2 = coord_2;
            coord_1= int(keypoints[0].pt.x)/848.0*-0.885;
            coord_2= int(keypoints[0].pt.y)/100.0*0.101;
        }else{
            coord_1 = prev1;
            coord_2 = prev2;
        }

    // Query the distance from the camera to the object in the center of the image
        float sum = 0;
        int num = 0;
    // Print the distance
    //cout<<coord_1<<","<<coord_2<<endl;
    xc.push_back(coord_1); //espera 250 sampletimes   
    yc.push_back(coord_2);
    shared->xcoord = coord_1;  // o algún valor generado
    shared->ycoord = coord_2;  // o algún valor generado
    shared->timestamp = std::chrono::steady_clock::now();
    shared->data_ready = true;

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end-start);
    dt.push_back(duration.count());
    }
    shm_unlink("/my_shared_mem");
    plot();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



