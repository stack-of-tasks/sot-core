/*
 * Copyright 2019,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <iostream>

#include <sot/core/sot.hh>
#include <sot/core/feature-generic.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/debug.hh>
#include <sot/core/task.hh>
#include <sot/core/gain-adaptive.hh>
#include <dynamic-graph/linear-algebra.h>
using namespace std;
using namespace dynamicgraph::sot;

class TestFeatureGeneric
{
 public:
  FeatureGeneric feature_,featureDes_;
  Task task_;
  int time_;
  int dim_,robotDim_,featureDim_;
  dynamicgraph::Vector manual_;
  
  TestFeatureGeneric(unsigned dim,std::string &name):
      feature_("feature"+name),
      featureDes_("featureDes"+name),
      task_("task"+name),
      time_(0)

  {
    feature_.setReference(&featureDes_);
    feature_.selectionSIN=Flags(true);

    task_.addFeature(feature_);
    task_.setWithDerivative(true);
    dim_ = dim;
    robotDim_ = featureDim_ = dim;

    dynamicgraph::Matrix Jq(dim,dim);
    Jq.setIdentity();
    feature_.jacobianSIN.setReference(&Jq);

    manual_.resize(dim);
  }

  void setInputs(dynamicgraph::Vector & s,
                 dynamicgraph::Vector &sd,
                 dynamicgraph::Vector &vd,
                 double gain)
  {
    feature_.errorSIN = s;
    feature_.errorSIN.access(time_);
    feature_.errorSIN.setReady();
    
    featureDes_.errorSIN = sd;
    featureDes_.errorSIN.access(time_);
    featureDes_.errorSIN.setReady();
    
    featureDes_.errordotSIN = vd;
    featureDes_.errordotSIN.access(time_);
    featureDes_.errordotSIN.setReady();
    
    task_.controlGainSIN = gain;
    task_.controlGainSIN.access(time_);
    task_.controlGainSIN.setReady();
  }

  void printInputs()
  {
    std::cout << "----- inputs -----" << std::endl;
    std::cout << "feature_.errorSIN: " << feature_.errorSIN(time_)
              << std::endl;
    std::cout << "featureDes_.errorSIN: " << featureDes_.errorSIN(time_)
              << std::endl;
    std::cout << "featureDes_.errordotSIN: " << featureDes_.errordotSIN(time_)
              << std::endl;
    std::cout << "task.controlGain: " << task_.controlGainSIN(time_)
              << std::endl;
  }

  void recompute()
  {
    
    feature_.errorSOUT.recompute(time_);
    feature_.errordotSOUT.recompute(time_);
    task_.taskSOUT.recompute(time_);
    task_.errorSOUT.recompute(time_);
    task_.errorTimeDerivativeSOUT.recompute(time_);    
    time_++;
    dynamicgraph::Vector s = feature_.errorSIN;
    dynamicgraph::Vector sd = featureDes_.errorSIN;
    dynamicgraph::Vector vd = featureDes_.errordotSIN;
    double gain = task_.controlGainSIN;
    dynamicgraph::Vector manual;
    for(unsigned int i=0;i<dim_;i++)
      manual_[i]  = vd[i] + gain*(sd(i)-s(i));
  }

  void printOutput()
  {
    std::cout << "----- output -----" << std::endl;
    std::cout << "time: " << time_ << std::endl;
    std::cout << "feature.errorSOUT: " << feature_.errorSOUT(time_)
              << std::endl;
    std::cout << "feature.errordotSOUT: " << feature_.errordotSOUT(time_)
              << std::endl;
    std::cout << "task.taskSOUT: " << task_.taskSOUT(time_)
              << std::endl;    
    //    std::cout << "task.errorSOUT: " << task_.errorSOUT(time_)
    //<< std::endl;
    //std::cout << "task.errordtSOUT: " << task_.errorTimeDerivativeSOUT(time_)
    //<< std::endl;    
    std::cout << "manual: " << manual_ << std::endl;
  }

  void runTest(dynamicgraph::Vector &s,
               dynamicgraph::Vector &sd,
               dynamicgraph::Vector &vd,
               double gain)
  {
    setInputs(s,sd,vd,gain);
    printInputs();
    recompute();
    printOutput();
  }
};


int main( void )
{
  std::string srobot("Robot");
  unsigned int dim=6;
  dynamicgraph::Vector s(dim), sd(dim);
  dynamicgraph::Vector vd(dim);
  double gain;
  
  TestFeatureGeneric testFeatureGeneric(dim,srobot);
  
  std::cout << " ----- Test Velocity -----" << std::endl;
  for(unsigned int i=0;i<dim;i++)
    s(i) = 0.0;
  
  for(unsigned int i=0;i<dim;i++)
    sd(i) = 0.0;

  for(unsigned int i=0;i<dim;i++)
    vd(i)=1.0;
  
  gain=0.0;
  
  testFeatureGeneric.runTest(s,sd,vd,gain);
  
  std::cout << " ----- Test Position -----" << std::endl;
  for(unsigned int i=0;i<dim;i++)
    s(i) = 0.0;
  
  for(unsigned int i=0;i<dim;i++)
    sd(i) = 2.0;
  
  for(unsigned int i=0;i<6;i++)
    vd(i)=0.0;
  
  gain=1.0;
  
  testFeatureGeneric.runTest(s,sd,vd,gain);

  std::cout << " ----- Test both -----" << std::endl;
  for(unsigned int i=0;i<dim;i++)
    s(i) = 0.0;
  
  for(unsigned int i=0;i<dim;i++)
    sd(i) = 2.0;
  
  for(unsigned int i=0;i<dim;i++)
    vd(i)=1.0;
  
  gain=3.0;
  
  testFeatureGeneric.runTest(s,sd,vd,gain);

  return 0;
}
