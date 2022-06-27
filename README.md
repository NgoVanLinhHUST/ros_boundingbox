# ros_boundingbox
bounding box vehicle





- link file data : file.bag , file video 
https://husteduvn-my.sharepoint.com/:f:/g/personal/linh_nv172653_sis_hust_edu_vn/EogaGA1vxiFFnz2EmsBcKaoBDT0TbVuaBWyuUf_qIfqRIw?e=M0PB9k


# Installing PCL tools and convert bag to pcd
$ sudo apt install pcl-tools


---   **convert file.bag to pcd **

$ cd catkin_ws/bagfile

$ rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>
 
Eg: rosrun pcl_ros bag_to_pcd 2017-06-08-15-49-45_0.bag /velodyne_points outputpcd/


- move folder outputpcd (file.pcd) to ../src/sensors/data/pcd/ 
 
 
 $ cd build 
 
 $ make 
 
 $ ./environment
