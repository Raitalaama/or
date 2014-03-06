
#include "or_common/bag_handler.h"

#include <pcl/io/savePNG>
#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Training objects");
  ros::rate(5);
  BagHandler bag_handler;
  bag_handler.setEveryThFrame(90);
  bag_handler.setInputDirectory("/media/DATA/Varasto/Japanidatat/objects_in_hand");
  bag_handler.loadBagsFromDir();

  int file_index = 0;
  while (ros::ok())
    {
      ros::spinOnce();
      std::cout << "Class: " << bag_handler.getCorrespondingClassName() << "\n";
      if (!bag_handler.getMsgsLeft())
        break;
      std::osstringstream file_name;
      file_name << "/home/raitalaama/or_output/pc_"<<i<<".png"; 
      pcl::io::savePNGFile(file_name.str(),*bag_handler.getNextPointCloud());
      rate.sleep();
    }

}
