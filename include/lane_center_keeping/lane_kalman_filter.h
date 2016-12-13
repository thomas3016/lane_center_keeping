
#ifndef INCLUDE_LANE_CENTER_KEEPING_CURBEDGEKALMANFILTER_H_
#define INCLUDE_LANE_CENTER_KEEPING_CURBEDGEKALMANFILTER_H_


/*
 * LaneKalmanFilter.h
 *
 *  Created on: Aug 10, 2016
 *      Author: aicrobo
 */

#include<opencv2/opencv.hpp>
#include<stdio.h>
#include<stdint.h>

using namespace std;
using namespace cv;

class CurbEdgeKalmanFilter
{

    public:
      CurbEdgeKalmanFilter(int stateNum,int measureNum)
      {

        stateNum_=stateNum;
        measureNum_=measureNum;
        kf.resize(2);
        state.resize(2);
        meas.resize(2);
        prediction.resize(2);


        kf[0]=new KalmanFilter(stateNum,measureNum,0);
        kf[1]=new KalmanFilter(stateNum,measureNum,0);
       // kf[0]->init(stateNum_,measureNum_,0);
        //kf[1]->init(stateNum_,measureNum_,0);

        state[0]=Mat(stateNum,1,CV_32FC1);
        state[1]=Mat(stateNum,1,CV_32FC1);
        meas[0]=Mat(measureNum,1,CV_32FC1);
        meas[1]=Mat(measureNum,1,CV_32FC1);

        prediction[0]=Mat(stateNum,1,CV_32FC1);
        prediction[1]=Mat(stateNum,1,CV_32FC1);
        kf[0]->transitionMatrix=Mat(stateNum,measureNum,CV_32FC1);
        kf[0]->measurementMatrix=Mat(stateNum,measureNum,CV_32FC1);
        kf[0]->processNoiseCov=Mat(stateNum,1,CV_32FC1);
        kf[0]->measurementNoiseCov=Mat(measureNum,1,CV_32FC1);

        kf[1]->transitionMatrix=Mat(stateNum,measureNum,CV_32FC1);
        kf[1]->measurementMatrix=Mat(stateNum,measureNum,CV_32FC1);
        kf[1]->processNoiseCov=Mat(stateNum,1,CV_32FC1);
        kf[1]->measurementNoiseCov=Mat(measureNum,1,CV_32FC1);

        right_wave_num=0;
        left_wave_num=0;

        left_non_exit=false;
        right_non_exit=false;
      }

      void init()
      {
          meas[0]=cv::Mat::zeros(measureNum_,1,CV_32FC1);
          meas[1]=cv::Mat::zeros(measureNum_,1,CV_32FC1);


          kf[0]->init(stateNum_,measureNum_,0);
          kf[1]->init(stateNum_,measureNum_,0);
          for(int i=0;i<2;i++)
          {
             setIdentity(kf[i]->transitionMatrix,Scalar::all(1));
             setIdentity(kf[i]->measurementMatrix,Scalar::all(1));
            // setIdentity(kf[i]->processNoiseCov,Scalar::all(1e-1));
             //setIdentity(kf[i]->measurementNoiseCov,Scalar(5/100,5/100));
             setIdentity(kf[i]->errorCovPost,Scalar::all(1));

             setIdentity(kf[i]->processNoiseCov,cvRealScalar(1e-3));
             setIdentity(kf[i]->measurementNoiseCov,cvRealScalar(1e-5));
             //setIdentity(kf->errorCovPost,Scalar::all(0.2));
          }
      }
      void set_Lane_Parameters(float a0,float a1,bool left_flag)
      {
        if(left_flag)
        {
            if(a0==0&&a1==0)
            {
              left_non_exit=true;
            }
           // else if(std::abs(a1-state[0].ptr<float>(0)[1])<1)
            else if(std::abs(a0)<2&&std::abs(a1)<1)
            {
              meas[0].ptr<float>(0)[0]=a0;
              meas[0].ptr<float>(1)[0]=a1;
            }
            else
            {
              left_wave_num=left_wave_num+1;
              meas[0].ptr<float>(0)[0]=a0;
              meas[0].ptr<float>(1)[0]=a1;
            }
        }
        else
        {
          if(a0==0&&a1==0)
          {
            right_non_exit=true;
          }
          else if(std::abs(a0)<2&&std::abs(a1)<1)
          {
            meas[1].ptr<float>(0)[0]=a0;
            meas[1].ptr<float>(1)[0]=a1;
          }
          else
          {
            right_wave_num=right_wave_num+1;
            meas[1].ptr<float>(0)[0]=a0;
            meas[1].ptr<float>(1)[0]=a1;
          }
        }
      }

      void InitPrediction(float a0,float a1,bool left_flag)
      {
        if(left_flag)
        {

          kf[0]->statePost.ptr<float>(0)[0]=a0;
          kf[0]->statePost.ptr<float>(1)[0]=a1;
          kf[0]->statePre.ptr<float>(0)[0]=a0;
          kf[0]->statePre.ptr<float>(1)[0]=a1;
        }
        else
        {

          kf[1]->statePost.ptr<float>(0)[0]=a0;
          kf[1]->statePost.ptr<float>(1)[0]=a1;
          kf[1]->statePre.ptr<float>(0)[0]=a0;
          kf[1]->statePre.ptr<float>(1)[0]=a1;
        }
      }

  void predict(bool left_flag)
  {
    if (left_flag)
    {
      if (left_non_exit)
      {
        left_non_exit = false;
        kf[0]->predict();
        kf[0]->statePost = kf[0]->statePre;
        state[0] = kf[0]->statePost;
      }
      else if (!left_non_exit && left_wave_num == 0)
      {
        kf[0]->predict();
        kf[0]->correct(meas[0]);
        state[0] = kf[0]->statePost;
      }
      else if (!left_non_exit && left_wave_num > 0 && left_wave_num <3)
      {
        kf[0]->predict();
        kf[0]->statePost = kf[0]->statePre;
        state[0] = kf[0]->statePost;
      }
      else if (!left_non_exit && left_wave_num >= 3)
      {
        left_wave_num = 0;
        kf[0]->statePost = meas[0];
        state[0] = kf[0]->statePost;
      }
    }
    else
    {
      if (right_non_exit)
      {
        right_non_exit = false;
        kf[1]->predict();
        kf[1]->statePost = kf[1]->statePre;
        state[1] = kf[1]->statePost;
      }

      else if (!right_non_exit && right_wave_num == 0)
      {

        kf[1]->predict();
        kf[1]->correct(meas[1]);
        state[1] = kf[1]->statePost;
      }
      else if (!right_non_exit && right_wave_num > 0 && right_wave_num <3)
      {

        kf[1]->predict();

        kf[1]->statePost = kf[1]->statePre;

        state[1] = kf[1]->statePost;

      }
      else if (!right_non_exit && right_wave_num >=3)
      {
        right_wave_num = 0;
        kf[1]->statePost = meas[1];
        state[1] = kf[1]->statePost;
      }
    }

   // std::cout << "measure data:" << meas[0].t() << "\t" << "predict data:" << kf[0]->statePre.t() << "\t" << "update data:" << kf[0]->statePost.t() << endl;
    //std::cout << "measure data:" << meas[1].t() << "\t" << "predict data:" << kf[1]->statePre.t() << "\t" << "update data:" << kf[1]->statePost.t() << endl;

  }

     vector<float>getStateL()
     {
        vector<float>ret;
        ret.push_back(state[0].ptr<float>(0)[0]);
        ret.push_back(state[0].ptr<float>(1)[0]);
        return ret;
     }
     vector<float>getStateR()
     {
       vector<float>ret;
       ret.push_back(state[1].ptr<float>(0)[0]);
       ret.push_back(state[1].ptr<float>(1)[0]);
       return ret;
     }

     ~CurbEdgeKalmanFilter()
     {
       kf.clear();
       meas.clear();
       state.clear();
       prediction.clear();
     }

    private:
        std::vector<KalmanFilter*> kf;
        std::vector<cv::Mat>meas;
        std::vector<cv::Mat>state;
        std::vector<cv::Mat>prediction;

        int left_wave_num;
        int right_wave_num;

        int stateNum_;
        int measureNum_;

        bool left_non_exit;
        bool right_non_exit;
};
#endif /* INCLUDE_LANE_CENTER_KEEPING_CURBEDGEKALMANFILTER_H_ */
