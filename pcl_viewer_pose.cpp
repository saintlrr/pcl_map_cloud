#include <iostream>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define NUM 10
#define DOWNSAMPLE_RATE 5

// This function displays the help
void
showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

// downsample frame
template <typename T>
std::vector<T> downSample(std::vector<T> vec, int rate = DOWNSAMPLE_RATE)
{
    std::vector<T> vec_new;
    typename std::vector<T>::iterator vec_iterator;
    for (vec_iterator = vec.begin()+1100; vec_iterator < vec.end()-150; vec_iterator += DOWNSAMPLE_RATE)
    {
        vec_new.push_back(*vec_iterator);
    }
    return vec_new;
}

// filter and downsample point cloud
template <typename PointT> void
downSample_filter (typename boost::shared_ptr<pcl::PointCloud<PointT> > source_cloud, 
    typename boost::shared_ptr<pcl::PointCloud<PointT> > source_cloud_filtered, std::string filter_type="None")
{
    clock_t start_time, end_time;
    start_time = clock();
    if (filter_type == "VoxelGrid"){
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(source_cloud);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(*source_cloud_filtered);
    }
    else if (filter_type == "Progressive")
    {
        pcl::PointIndicesPtr ground (new pcl::PointIndices);
        pcl::ProgressiveMorphologicalFilter<PointT> pmf;
        pmf.setMaxWindowSize(20);
        pmf.setSlope(1.0f);
        pmf.setInitialDistance(0.5f);
        pmf.setInputCloud(source_cloud);
        pmf.setMaxDistance(3.0f);
        pmf.extract(ground->indices);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (source_cloud);
        extract.setIndices (ground);
        extract.filter (*source_cloud_filtered);
    }
    else if (filter_type == "SACSeg")
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr ground (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (source_cloud);
        seg.segment (*ground, *coefficients);

         pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (source_cloud);
        extract.setIndices (ground);
        extract.filter (*source_cloud_filtered);
    }
    else if (filter_type == "None")
    {
        *source_cloud_filtered = *source_cloud;
    }
    end_time = clock();
    std::cout << filter_type <<" downsample filter Time : " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
}

// outlier remove point cloud
template <typename PointT> void
outlier_filter (typename boost::shared_ptr<pcl::PointCloud<PointT> > source_cloud, 
    typename boost::shared_ptr<pcl::PointCloud<PointT> > source_cloud_filtered, std::string filter_type="None")
{
    clock_t start_time, end_time;
    start_time = clock();
    if (filter_type == "Statistical"){
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud (source_cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*source_cloud_filtered);
    }
    else if (filter_type == "None"){
        *source_cloud_filtered = *source_cloud;
    }
    end_time = clock();
    std::cout << filter_type <<" outlier filter Time : " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
}

// This is the main function
int
main (int argc, char** argv)
{

    // Show help
    if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
        showHelp (argv[0]);
        return 0;
    }   

    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    bool file_is_pcd = false;

    struct stat s;
    if( stat(argv[1],&s) == 0 )
    {
        if( s.st_mode & S_IFDIR )
        {
            DIR *dir;
            struct dirent *ent;
            int frame_idx = 0;
            if ((dir = opendir(argv[1])) != NULL)
            {
                struct stat s2;
                // std::vector<std::vector<double> > pose_vecs(0, std::vector<double> (8));
                std::vector<std::vector<double> > pose_vecs;
                std::vector<double> time_list;

                clock_t startTime,endTime;
	            startTime = clock();

                // read pose file
                if (stat(argv[2], &s2) == 0)
                {
                    if(s2.st_mode & S_IFREG )
                    {
                        std::string line;
                        ifstream pose_file (argv[2]);
                        // int line_idx = 0;
                        // int pose_ele_idx = 0;
                        if (pose_file.is_open())
                        {
                            while ( getline (pose_file,line) )
                            {
                                // std::cout << line << '\n';
                                std::string  pose_ele_string;
                                std::istringstream tmp(line);
                                std::vector<double> pose_vec;
                                // pose_vecs.resize(line_idx+1);
                                while( getline(tmp, pose_ele_string, ' '))
                                {
                                    std::string::size_type sz;
                                    double pose_ele = std::stold(pose_ele_string, &sz);
                                    // std::cout << pose_ele <<std::endl;
                                    // pose_vecs[line_idx].resize(8);
                                    // pose_vecs[line_idx][pose_ele_idx] = pose_ele;
                                    pose_vec.push_back(pose_ele);
                                    // if (pose_ele_idx == 3){
                                    //     time_list.push_back(pose_ele);
                                    // }
                                    // pose_ele_idx += 1;
                                }
                                time_list.push_back(pose_vec[3]);
                                pose_vecs.push_back(pose_vec);
                                // pose_ele_idx = 0;
                                // line_idx += 1;
                            }
                            pose_file.close();

                            time_list = downSample(time_list);
                            pose_vecs = downSample(pose_vecs);

                            // cout the pose data
                            // for (int i = 0; i < pose_vecs.size(); i++)
                            // {
                            //     for(int j = 0; j < pose_vecs[i].size(); j++){
                            //         std::cout.precision(16);
                            //         std::cout << pose_vecs [i][j] << ' ';
                            //     }
                            //     std::cout << std::endl;
                            // }
                            // for (int i = 0; i < time_list.size(); i++)
                            // {
                            //     std::cout.precision(16);
                            //     std::cout << time_list[i] << ' ';
                            //     std::cout << std::endl;
                            // }
                        }
                        else std::cout << "Unable to open " << argv[2] << " file"; 
                    }
                }
                else 
                {
                    std::cout << "please give the pose data." <<std::endl;
                    showHelp (argv[0]);
                    return -1;
                }

                endTime = clock();
                std::cout << "Pose data load Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

                // the first point transformer
                Eigen::Quaterniond q0;
                Eigen::Affine3d T0 = Eigen::Affine3d::Identity();
                Eigen::Vector3d t0(pose_vecs[0][0], pose_vecs[0][1], pose_vecs[0][2]);
                q0.w() = pose_vecs[0][4];
                q0.x() = pose_vecs[0][5];
                q0.y() = pose_vecs[0][6];
                q0.z() = pose_vecs[0][7];
                T0.rotate(q0);
                // T0.translation() << (0,0,0);
                T0.translation() = t0;

                // Visualization
                pcl::visualization::PCLVisualizer viewer ("pose pointcloud");
                viewer.addCoordinateSystem (1.0, "cloud", 0);
                viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
                pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
                // read pcd files
                int count = 0;
                while((ent = readdir(dir)) != NULL)
                {
                    if (count <NUM){
                    std::string file_name = std::string(ent->d_name);
                    if (file_name.find(".pcd") != std::string::npos){

                        //find timestamp index
                        std::size_t found_idx = file_name.find(".pcd");
                        std::string  timestamp_string = file_name.substr(0, found_idx);
                        std::string::size_type sz;
                        double timestamp = std::stold(timestamp_string, &sz);
                        // std::cout.precision(16);
                        // std::cout << timestamp << std::endl;
                        std::vector<double>::iterator it;
                        it = find (time_list.begin(), time_list.end(), timestamp);
                        if (it != time_list.end()){
                            frame_idx = it - time_list.begin();
                            std::cout << "frame_idx : " << frame_idx << std::endl;}
                        else {
                            std::cout << "timestamp not found in time_list\n";
                            continue;}

                        // change to current frame to first frame transform, inverse maybe use another way
                        Eigen::Quaterniond q;
                        Eigen::Affine3d T = Eigen::Affine3d::Identity();
                        Eigen::Affine3d Ts = Eigen::Affine3d::Identity();
                        Eigen::Vector3d t(pose_vecs[frame_idx][0], pose_vecs[frame_idx][1], pose_vecs[frame_idx][2]);
                        q.w() = pose_vecs[frame_idx][4];
                        q.x() = pose_vecs[frame_idx][5];
                        q.y() = pose_vecs[frame_idx][6];
                        q.z() = pose_vecs[frame_idx][7];
                        std::cout<< q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z() <<std::endl;
                        T.rotate(q);
                        T.translation() = t;
                        Ts = T0.inverse()*T;
                        std::cout << "Ts translation: " << Ts.translation() << std::endl;

                        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

                        if (pcl::io::loadPCDFile (argv[1] + file_name, *source_cloud) < 0)  {
                            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
                            showHelp (argv[0]);
                            return -1;
                        }

                        // Filter the point cloud
                        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI> ());

                        downSample_filter (source_cloud, source_cloud_filtered, "None");

                        // outlier removal
                        outlier_filter (source_cloud_filtered, source_cloud_filtered, "None");

                        // print num of points in pointclouds
                        std::cout << "PointCloud before filtering: " << source_cloud->width * source_cloud->height 
                            << " data points (" << pcl::getFieldsList (*source_cloud) << ").";
                        std::cout << "PointCloud after filtering: " << source_cloud_filtered->width * source_cloud_filtered->height 
                            << " data points (" << pcl::getFieldsList (*source_cloud_filtered) << ")." << std::endl;

                        // output point cloud
                        // for (pcl::PointCloud<pcl::PointXYZI>::iterator ii = (*source_cloud_filtered).begin(); ii<(*source_cloud_filtered).end() ;
                        //     ii++)
                        //     {
                        //         std::cout << ii->x <<" " <<ii->y<< " "<<ii->z <<" "<<ii->intensity <<std::endl;
                        //     }

                        // Executing the transformation
                        clock_t start_time, end_time;
                        start_time = clock();

                        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());                       
                        
                        pcl::transformPointCloud (*source_cloud_filtered, *transformed_cloud, Ts);

                        *map_cloud = *map_cloud + *transformed_cloud;

                        end_time = clock();
                        std::cout << "transform Time : " <<(double)(start_time - end_time) / CLOCKS_PER_SEC << "s" << std::endl;


                        // pcl::visualization::PCLVisualizer viewer ("pose pointcloud");

                        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
                        // viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud" + file_name);
                        // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, "transformed_cloud" + file_name);

                        // viewer.addCoordinateSystem (1.0, "cloud", 0);
                        // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
                        // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, "transformed_cloud");
                        // //viewer.setPosition(800, 400); // Setting visualiser window position

                        // while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
                        //     viewer.spinOnce ();
                        // }

                        printf ("%s\n", ent->d_name);
                        std::cout << "count :" << count << std::endl;
                        count += 1;
                    }
                    }
                }
                //viewer.setPosition(800, 400); // Setting visualiser window position
                // viewer.initCameraParameters();

                // Filter the point cloud
                pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI> ());

                downSample_filter (map_cloud, map_cloud_filtered, "None");

                std::cout << "PointCloud before filtering: " << map_cloud->width * map_cloud->height 
                    << " data points (" << pcl::getFieldsList (*map_cloud) << ").";
                std::cout << "PointCloud after filtering: " << map_cloud_filtered->width * map_cloud_filtered->height 
                    << " data points (" << pcl::getFieldsList (*map_cloud_filtered) << ")." << std::endl;

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_cloud_color_handler (map_cloud_filtered, 230, 230, 230);
                viewer.addPointCloud (map_cloud_filtered, map_cloud_color_handler, "map_cloud");
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, "map_cloud");

                endTime = clock();
                std::cout << "Total Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

                while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
                    viewer.spinOnce ();
                }
            } else {
                /* could not open directory */
                perror ("");
                return EXIT_FAILURE;
            }   
            closedir(dir);
        }
        else if( s.st_mode & S_IFREG )
        {
            filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

            if (filenames.size () != 1)  {
                filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

                if (filenames.size () != 1) {
                showHelp (argv[0]);
                return -1;
                } else {
                file_is_pcd = true;
                }
            }

            // Load file | Works with PCD and PLY files
            pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

            if (file_is_pcd) {
                if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
                std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
                showHelp (argv[0]);
                return -1;
                }
            } else {
                if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
                std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
                showHelp (argv[0]);
                return -1;
                }
            }

            /* Reminder: how transformation matrices work :

                    |-------> This column is the translation
                | 1 0 0 x |  \
                | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
                | 0 0 1 z |  /
                | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

                METHOD #1: Using a Matrix4f
                This is the "manual" method, perfect to understand but error prone !
            */
            Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

            // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
            float theta = M_PI/4; // The angle of rotation in radians
            transform_1 (0,0) = cos (theta);
            transform_1 (0,1) = -sin(theta);
            transform_1 (1,0) = sin (theta);
            transform_1 (1,1) = cos (theta);
            //    (row, column)

            // Define a translation of 2.5 meters on the x axis.
            transform_1 (0,3) = 2.5;

            // Print the transformation
            printf ("Method #1: using a Matrix4f\n");
            std::cout << transform_1 << std::endl;

            /*  METHOD #2: Using a Affine3f
                This method is easier and less error prone
            */
            Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

            // Define a translation of 2.5 meters on the x axis.
            transform_2.translation() << 2.5, 0.0, 0.0;

            // The same rotation matrix as before; theta radians around Z axis
            transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

            // Print the transformation
            printf ("\nMethod #2: using an Affine3f\n");
            std::cout << transform_2.matrix() << std::endl;

            // Executing the transformation
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
            // You can either apply transform_1 or transform_2; they are the same
            pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);

            // Visualization
            printf(  "\nPoint cloud colors :  white  = original point cloud\n"
                "                        red  = transformed point cloud\n");
            pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

            // Define R,G,B colors for the point cloud
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_cloud_color_handler (source_cloud, 255, 255, 255);
            // We add the point cloud to the viewer and pass the color handler
            viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
            viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

            viewer.addCoordinateSystem (1.0, "cloud", 0);
            viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
            //viewer.setPosition(800, 400); // Setting visualiser window position

            while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
                viewer.spinOnce ();
            }

            return 0;
        }
        else
        {
            std::cerr << "error no dir or file" << std::endl;//something else
            return -1;
        }
    }
    else
    {
        std::cerr << "error stat" << std::endl;//error
        return -1;
    }
}