#include "or_common/bag_handler.h"
#include "or_trainer/or_trainer.h"
#include "or_trainer/segmenter.h"

#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Training objects");
  ros::NodeHandle nh("or_trainer");

  or_training::ORTrainer or_training;
  or_training.train();


}

