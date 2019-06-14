#include <pcl_viewer_pose/pcl_viewer_pose.h>

using namespace pcl_viewer_pose;
// This is the main function
int main(int argc, char **argv)
{
    // Show help
    if (pcl::console::find_switch(argc, argv, "-h") || 
        pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        return 0;
    }

    // read Config
    readConfig();

    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    bool file_is_pcd = false;

    struct stat arg_1;
    // check if argv[1] is given
    if (stat(argv[1], &arg_1) == 0)
    {   // check if argv[1] is directory
        if (arg_1.st_mode & S_IFDIR)
        {
            DIR *dir;
            struct dirent *ent;
            int frame_idx = 0;
            dir = opendir(argv[1]);

            struct stat arg_2;
            // std::vector<std::vector<double> > pose_vecs(0, std::vector<double> (8));
            std::vector<std::vector<double>> pose_vecs;
            std::vector<double> time_list;

            clock_t startTime, endTime;
            startTime = clock();

            // read pose file
            // check if argv2 is given
            if (stat(argv[2], &arg_2) == 0)
            {   // check if argv2 is a regular file
                if (arg_2.st_mode & S_IFREG)
                {
                    std::string line;
                    ifstream pose_file(argv[2]);
                    int line_idx = 0;
                    // int pose_ele_idx = 0;
                    if (pose_file.is_open())
                    {
                        while (getline(pose_file, line))
                        {
                            // std::cout << line << '\n';
                            std::string pose_ele_string;
                            std::istringstream tmp(line);
                            std::vector<double> pose_vec;
                            // pose_vecs.resize(line_idx+1);
                            while (getline(tmp, pose_ele_string, ' '))
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
                            if(NAME_FLAG == "time")
                                time_list.push_back(pose_vec[3]);
                            else if(NAME_FLAG == "seq")
                                time_list.push_back(line_idx + 1);
                            pose_vecs.push_back(pose_vec);
                            // pose_ele_idx = 0;
                            line_idx += 1;
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
                    else
                    {
                        std::cerr << "Unable to open " << argv[2] << " file"
                                  << std::endl;
                        showHelp(argv[0]);
                        return -1;
                    }
                        
                }
                else
                {
                    std::cerr << "the pose data file is not a regular file" 
                              << std::endl;
                    showHelp(argv[0]);
                    return -1;
                }
                
            }
            else
            {
                std::cout << "please give the pose data." << std::endl;
                showHelp(argv[0]);
                return -1;
            }

            endTime = clock();
            std::cout << "Pose data load Time : " 
                        << (double)(endTime - startTime) / CLOCKS_PER_SEC 
                        << "s" << std::endl;

            // the first point transformer
            Eigen::Quaterniond q0;
            Eigen::Affine3d T0 = Eigen::Affine3d::Identity();
            Eigen::Vector3d t0(pose_vecs[0][0], pose_vecs[0][1], pose_vecs[0][2]);
            q0.w() = pose_vecs[0][4];
            q0.x() = pose_vecs[0][5];
            q0.y() = pose_vecs[0][6];
            q0.z() = pose_vecs[0][7];
            // Eigen::Vector3d t0(pose_vecs[0][2], pose_vecs[0][3], pose_vecs[0][4]);
            // q0.w() = pose_vecs[0][8];
            // q0.x() = pose_vecs[0][5];
            // q0.y() = pose_vecs[0][6];
            // q0.z() = pose_vecs[0][7];
            T0.rotate(q0);
            // T0.translation() << (0,0,0);
            T0.translation() = t0;

            // Visualization
            pcl::visualization::PCLVisualizer viewer("pose pointcloud");
            viewer.addCoordinateSystem(1.0, "cloud", 0);
            viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); 
            pcl::PointCloud<pcl::PointXYZI>::Ptr 
                map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr 
                pre_cloud(new pcl::PointCloud<pcl::PointXYZI>());

            // record before and after scan matching poses into files
            ofstream pose_pre;
            ofstream pose_post;
            pose_pre.open("pose_pre.txt");
            pose_post.open("pose_post.txt");

            // read pcd files
            int count = 0;
            std::vector<std::string> file_name_list;
            std::vector<double> timestamp_list;
            std::vector<size_t> idx_list;
            while ((ent = readdir(dir)) != NULL)
            {
                std::string file_name = std::string(ent->d_name);
                if (file_name.find(".pcd") != std::string::npos)
                {
                    file_name_list.push_back(file_name);
                    //find timestamp or sequence index
                    std::string timestamp_string = file_name.substr(0, 
                        file_name.find(".pcd"));
                    std::string::size_type sz;
                    double timestamp = std::stold(timestamp_string, &sz);
                    timestamp_list.push_back(timestamp);
                }
            }
            // sort filename list
            idx_list = sort_index(timestamp_list);
            for (std::vector<size_t>::iterator idx = idx_list.begin(); 
                 idx<idx_list.end() && count < NUM; idx++)
            {
                double timestamp = timestamp_list[*idx];
                std::string file_name = file_name_list[*idx];
                std::vector<double>::iterator it;
                it = find(time_list.begin(), time_list.end(), timestamp);
                if (it != time_list.end())
                {
                    frame_idx = it - time_list.begin();
                    std::cout << "frame_idx : " << frame_idx << std::endl;
                    std::cout << file_name << std::endl;
                    std::cout << "count :" << count << std::endl;
                }
                else continue;
                // change to current frame to first frame transform, 
                Eigen::Quaterniond q;
                Eigen::Affine3d T = Eigen::Affine3d::Identity();
                Eigen::Affine3d Ts = Eigen::Affine3d::Identity();
                Eigen::Affine3d Tf = Eigen::Affine3d::Identity();
                // current frame transformation
                Eigen::Vector3d t(pose_vecs[frame_idx][0], 
                                    pose_vecs[frame_idx][1], 
                                    pose_vecs[frame_idx][2]);
                q.w() = pose_vecs[frame_idx][4];
                q.x() = pose_vecs[frame_idx][5];
                q.y() = pose_vecs[frame_idx][6];
                q.z() = pose_vecs[frame_idx][7];
                // Eigen::Vector3d t(pose_vecs[frame_idx][2], 
                //                   pose_vecs[frame_idx][3], 
                //                   pose_vecs[frame_idx][4]);
                // q.w() = pose_vecs[frame_idx][8];
                // q.x() = pose_vecs[frame_idx][5];
                // q.y() = pose_vecs[frame_idx][6];
                // q.z() = pose_vecs[frame_idx][7];
                std::cout << "quaternion square :" 
                            << q.w() * q.w() + q.x() * q.x() + 
                                q.y() * q.y() + q.z() * q.z() 
                            << std::endl;
                T.rotate(q);
                T.translation() = t;
                Ts = T0.inverse() * T;
                std::cout << "Ts translation: " << Ts.translation() 
                            << std::endl;

                pcl::PointCloud<pcl::PointXYZI>::Ptr 
                    source_cloud(new pcl::PointCloud<pcl::PointXYZI>());

                // load pcd file
                if (pcl::io::loadPCDFile(argv[1] + file_name, *source_cloud) < 0)
                {
                    std::cout << "Error loading point cloud " 
                                << argv[filenames[0]] << std::endl;
                    showHelp(argv[0]);
                    return -1;
                }
                std::cout << "read pcd file successfult" << std::endl;

                // Filter the point cloud
                pcl::PointCloud<pcl::PointXYZI>::Ptr 
                    source_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

                downSample_filter(source_cloud, source_cloud_filtered, FILTER_TYPE);

                // outlier removal
                outlier_filter(source_cloud_filtered, source_cloud_filtered, OUTLIER_FILTER_TYPE);

                // print num of points in pointclouds
                std::cout << "PointCloud before filtering: " 
                            << source_cloud->width * source_cloud->height
                            << " data points (" << pcl::getFieldsList(*source_cloud) << ").";
                std::cout << "PointCloud after filtering: " 
                            << source_cloud_filtered->width * source_cloud_filtered->height
                            << " data points (" 
                            << pcl::getFieldsList(*source_cloud_filtered) 
                            << ")." << std::endl;

                // output point cloud
                // for (pcl::PointCloud<pcl::PointXYZI>::iterator ii = 
                //         (*source_cloud_filtered).begin(); 
                //     ii<(*source_cloud_filtered).end();
                //     ii++)
                //     {
                //         std::cout << ii->x <<" " <<ii->y<< " "<<ii->z <<" "
                //                   <<ii->intensity <<std::endl;
                //     }

                // Executing the transformation
                clock_t start_time, end_time;
                start_time = clock();

                pcl::PointCloud<pcl::PointXYZI>::Ptr 
                    transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());

                pcl::transformPointCloud(*source_cloud_filtered, *transformed_cloud, Ts);

                pose_pre << Ts.translation() << std::endl;
                if (count > 0)
                {
                    if (MATCH_FLAG)
                    {
                        std::cout << "start to do scan matching " 
                                    << std::endl;
                        Tf = ScanMatching(transformed_cloud, pre_cloud, SCAN_TYPE);
                    }
                    *pre_cloud = *transformed_cloud;
                    *map_cloud = *map_cloud + *transformed_cloud;
                    Ts = Tf * Ts;
                    pose_post << Ts.translation() << std::endl;
                }
                else if (count == 0)
                {
                    *pre_cloud = *transformed_cloud;
                    *map_cloud = *map_cloud + *transformed_cloud;
                    pose_post << Ts.translation() << std::endl;
                }
                // *map_cloud = *map_cloud + *transformed_cloud;

                end_time = clock();
                std::cout << "transform Time : " 
                            << (double)(end_time - start_time) / CLOCKS_PER_SEC 
                            << "s" << std::endl;

                // pcl::visualization::PCLVisualizer viewer ("pose pointcloud");

                // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> 
                //     transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
                // viewer.addPointCloud (transformed_cloud, 
                //                       transformed_cloud_color_handler, 
                //                       "transformed_cloud" + file_name);
                // viewer.setPointCloudRenderingProperties (
                //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                //     0.01, "transformed_cloud" + file_name);

                // viewer.addCoordinateSystem (1.0, "cloud", 0);
                // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
                // viewer.setPointCloudRenderingProperties (
                //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                //     0.01, "transformed_cloud");
                // //viewer.setPosition(800, 400); // Setting visualiser window position

                // while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
                //     viewer.spinOnce ();
                // }

                count++;
            }
            // viewer.setPosition(800, 400); // Setting visualiser window position
            // viewer.initCameraParameters();

            // close files
            pose_pre.close();
            pose_post.close();

            // Filter the point cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr 
                map_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

            downSample_filter(map_cloud, map_cloud_filtered, FILTER_TYPE);

            // outlier removal
            outlier_filter(map_cloud_filtered, map_cloud_filtered, OUTLIER_FILTER_TYPE);

            std::cout << "PointCloud before filtering: " 
                        << map_cloud->width * map_cloud->height
                        << " data points (" << pcl::getFieldsList(*map_cloud) << ").";
            std::cout << "PointCloud after filtering: " 
                        << map_cloud_filtered->width * map_cloud_filtered->height
                        << " data points (" << pcl::getFieldsList(*map_cloud_filtered) 
                        << ")." << std::endl;
            if(COLOR_FLAG){
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                map_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>());
                addColorByHeight(map_cloud_filtered, map_cloud_colored);
                pcl::visualization::PointCloudColorHandlerRGBField<
                    pcl::PointXYZRGB> rgb(map_cloud_colored);
                viewer.addPointCloud(
                    map_cloud_colored, rgb, "map_cloud");
                viewer.setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, "map_cloud");
            } else{
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> 
                map_cloud_color_handler(map_cloud_filtered, 230, 230, 230);
            viewer.addPointCloud(
                map_cloud_filtered, map_cloud_color_handler, "map_cloud");
            viewer.setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, "map_cloud");
            }

            endTime = clock();
            std::cout << "Total Time : " 
                        << (double)(endTime - startTime) / CLOCKS_PER_SEC 
                        << "s" << std::endl;

            while (!viewer.wasStopped())
            { // Display the visualiser until 'q' key is pressed
                viewer.spinOnce();
            }

            closedir(dir);
        }
        else if (arg_1.st_mode & S_IFREG)
        {
            filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

            if (filenames.size() != 1)
            {
                filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

                if (filenames.size() != 1)
                {
                    showHelp(argv[0]);
                    return -1;
                }
                else
                {
                    file_is_pcd = true;
                }
            }

            // Load file | Works with PCD and PLY files
            pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());

            if (file_is_pcd)
            {
                if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
                {
                    std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                              << std::endl;
                    showHelp(argv[0]);
                    return -1;
                }
            }
            else
            {
                if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
                {
                    std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                              << std::endl;
                    showHelp(argv[0]);
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
            float theta = M_PI / 4; // The angle of rotation in radians
            transform_1(0, 0) = cos(theta);
            transform_1(0, 1) = -sin(theta);
            transform_1(1, 0) = sin(theta);
            transform_1(1, 1) = cos(theta);
            //    (row, column)

            // Define a translation of 2.5 meters on the x axis.
            transform_1(0, 3) = 2.5;

            // Print the transformation
            printf("Method #1: using a Matrix4f\n");
            std::cout << transform_1 << std::endl;

            /*  METHOD #2: Using a Affine3f
                This method is easier and less error prone
            */
            Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

            // Define a translation of 2.5 meters on the x axis.
            transform_2.translation() << 2.5, 0.0, 0.0;

            // The same rotation matrix as before; theta radians around Z axis
            transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

            // Print the transformation
            printf("\nMethod #2: using an Affine3f\n");
            std::cout << transform_2.matrix() << std::endl;

            // Executing the transformation
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            // You can either apply transform_1 or transform_2; they are the same
            pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

            // Visualization
            printf("\nPoint cloud colors :  white  = original point cloud\n"
                   "                        red  = transformed point cloud\n");
            pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

            // Define R,G,B colors for the point cloud
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_cloud_color_handler(source_cloud, 255, 255, 255);
            // We add the point cloud to the viewer and pass the color handler
            viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
            viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

            viewer.addCoordinateSystem(1.0, "cloud", 0);
            viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
            //viewer.setPosition(800, 400); // Setting visualiser window position

            while (!viewer.wasStopped())
            { // Display the visualiser until 'q' key is pressed
                viewer.spinOnce();
            }

            return 0;
        }
        else
        {
            std::cerr << "error no dir or file" << std::endl; //something else
            return -1;
        }
    }
    else
    {
        std::cerr << "error stat" << std::endl; //error
        showHelp(argv[0]);
        return -1;
    }
}