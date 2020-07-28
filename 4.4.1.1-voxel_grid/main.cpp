#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h> 
int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}
	//remove NAN
	// Filter object.
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	// We set the size of every voxel to be 1x1x1cm
	// (only one point per every cubic centimeter will survive).
	filter.setLeafSize(0.01f, 0.01f, 0.01f);

	filter.filter(*filteredCloud);
	pcl::io::savePCDFileASCII("voxel_grid.pcd",*filteredCloud);
  cout<<"cloud"<<cloud->points.size()<<"filteredCloud"<<filteredCloud->points.size()<<endl;
    pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer(argc, argv, "example");
	int vp_1, vp_2;
	p->setBackgroundColor(0, 0, 0);
	p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);
	p->removePointCloud("source");
	p->removePointCloud("target");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>	cloud_tgt_h(cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_src_h(filteredCloud, 255, 0, 0);
	p->addPointCloud(cloud, cloud_tgt_h, "target", vp_1);
	p->addPointCloud(filteredCloud, cloud_src_h, "source", vp_2);
	p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
	p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
	p->spin();
	p->removePointCloud("source");
	p->removePointCloud("target");
}
