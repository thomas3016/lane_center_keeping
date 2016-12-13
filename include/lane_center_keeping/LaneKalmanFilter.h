/*
 * LaneKalmanFilter.h
 *
 *  Created on: Sep 28, 2016
 *      Author: aicrobo
 */

#ifndef INCLUDE_LANE_CENTER_KEEPING_LANEKALMANFILTER_H_
#define INCLUDE_LANE_CENTER_KEEPING_LANEKALMANFILTER_H_


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
class LaneKalmanFilter
{
    public:
      LaneKalmanFilter(Size s)
      {
        size_=s;
        //CvKalman*kalman=cvCreatekalman(stateNum,measureNum,0);

        kf=new KalmanFilter(4,4,0);
        state=Mat(4,1,CV_32FC1);
        meas=Mat(4,1,CV_32FC1);
        prediction=Mat(4,1,CV_32FC1);

        kf->transitionMatrix=Mat(4,4,CV_32FC1);
        kf->measurementMatrix=Mat(4,4,CV_32FC1);
        setIdentity(kf->transitionMatrix);
        setIdentity(kf->measurementMatrix);
        setIdentity(kf->processNoiseCov,Scalar::all(1e-1));
        setIdentity(kf->measurementNoiseCov,Scalar(16/100,15/100,5/100,5/100));
        //setIdentity(kf->measurementNoiseCov,Scalar(26/100,25/100,15/100,15/100));
        setIdentity(kf->errorCovPost,Scalar::all(1));

      }

    private:
      int stateNum_;
      int measureNum_;

    protected:
        Size size_;
        //Point stateLaneR,stateLaneL;
        //Point2f measLaneL,measLaneR;
        Mat state,prediction,meas;
        KalmanFilter*kf;
        bool m_bprior;
        int left_wave_num;
        int right_wave_num;

       // bool m_bMeasurementAvail;

    public:
     LaneKalmanFilter(int stateNum,int measureNum)
    {
      stateNum_=stateNum;
      measureNum=stateNum;
      kf=new KalmanFilter(stateNum,measureNum,0);

      state=Mat(stateNum,1,CV_32FC1);
      meas=Mat(measureNum,1,CV_32FC1);
      prediction=Mat(stateNum,1,CV_32FC1);

      //CvMat*process_noise=cvCreateMat(stateNum,1,CV_32FC1);
      //CvMat*measurement=cvCreateMat(measureNum,1,CV_32FC1);
      kf->processNoiseCov=Mat(4,1,CV_32FC1);
      kf->measurementNoiseCov=Mat(4,1,CV_32FC1);
      kf->transitionMatrix=Mat(4,4,CV_32FC1);
      kf->measurementMatrix=Mat(4,4,CV_32FC1);

      left_wave_num=0;
      right_wave_num=0;
    }
     ~LaneKalmanFilter()
     {
       if(kf!=NULL)
       {
         kf=NULL;
       }

     }

      void init()
      {
        m_bprior=true;

        meas=cv::Mat::zeros(4,1,CV_32FC1);

        kf->init(4,4,0);

        setIdentity(kf->transitionMatrix,Scalar::all(1));
        setIdentity(kf->measurementMatrix,Scalar::all(1));

        setIdentity(kf->processNoiseCov,cvRealScalar(1e-7));
        setIdentity(kf->measurementNoiseCov,cvRealScalar(1e-5));
        setIdentity(kf->errorCovPost,Scalar::all(0.2));
      }

  void InitPrediction(Point2f p1,Point2f p2)
  {
    m_bprior=false;
    /*
    kf->statePost.ptr<float>(0)[0]=p1.x;
    kf->statePost.ptr<float>(1)[0]=p2.x;
    state.ptr<float>(0)[0]=p1.x;
    state.ptr<float>(1)[0]=p2.x;
    */

      kf->statePost.ptr<float>(0)[0]=p1.y;
      kf->statePost.ptr<float>(1)[0]=p2.y;
      state.ptr<float>(0)[0]=p1.y;
      state.ptr<float>(1)[0]=p2.y;
  }

  vector<Point2f> Predict(bool left_flag)
  {
     prediction=kf->predict();
     vector<Point2f>ret;
     if(left_flag)
     {
       //ret.push_back(Point2f(prediction.ptr<float>(0)[0],-20));
       //ret.push_back(Point2f(prediction.ptr<float>(1)[0],30));
       ret.push_back(Point2f(-5,prediction.ptr<float>(0)[0]));
       ret.push_back(Point2f(15,prediction.ptr<float>(1)[0]));
     }
     else
     {
      // ret.push_back(Point2f(prediction.ptr<float>(2)[0],-20));
      // ret.push_back(Point2f(prediction.ptr<float>(3)[0],30));

       ret.push_back(Point2f(-5,prediction.ptr<float>(2)[0]));
       ret.push_back(Point2f(15,prediction.ptr<float>(3)[0]));
     }
     return ret;

  }

  vector<Point2f> Correct(Point2f p1,Point2f p2)
  {

    vector<Point2f>ret;
    if(m_bprior)
     {
       this->InitPrediction(p1,p2);
       vector<Point2f>ret;
      // ret.push_back(Point2f(state.ptr<float>(0)[0],-20));
      // ret.push_back(Point2f(state.ptr<float>(1)[0],30));
       ret.push_back(Point2f(-5,state.ptr<float>(0)[0]));
       ret.push_back(Point2f(15,state.ptr<float>(1)[0]));
       return ret;
     }
     prediction=kf->predict();
     //since new detections are available,update the new positions

    // meas.ptr<float>(0)[0]=p1.x;
    // meas.ptr<float>(1)[0]=p2.x;
     meas.ptr<float>(0)[0]=p1.y;
     meas.ptr<float>(1)[0]=p2.y;

     //adjust kalman
     kf->correct(meas);
     //calculate new state
    // ret.push_back(Point2f(kf->statePost.ptr<float>(0)[0],-20));
    // ret.push_back(Point2f(kf->statePost.ptr<float>(1)[0],30));
     ret.push_back(Point2f(-5,kf->statePost.ptr<float>(0)[0]));
     ret.push_back(Point2f(15,kf->statePost.ptr<float>(1)[0]));
     return ret;

  }


      void setStateLaneL(Point2f p1,Point2f p2)
      {
        //state.ptr<float>(0)[0]=p1.x;
        //state.ptr<float>(1)[0]=p2.x;

        //kf->statePost.ptr<float>(0)[0]=p1.x;
        //kf->statePost.ptr<float>(1)[0]=p2.x;

        state.ptr<float>(0)[0]=p1.y;
        state.ptr<float>(1)[0]=p2.y;

        kf->statePost.ptr<float>(0)[0]=p1.y;
        kf->statePost.ptr<float>(1)[0]=p2.y;

      }
      void setStateLaneR(Point2f p1,Point2f p2 )
      {
        //state.ptr<float>(2)[0]=p1.x;
        //state.ptr<float>(3)[0]=p2.x;
        //kf->statePost.ptr<float>(2)[0]=p1.x;
        //kf->statePost.ptr<float>(3)[0]=p2.x;

        state.ptr<float>(2)[0]=p1.y;
        state.ptr<float>(3)[0]=p2.y;
        kf->statePost.ptr<float>(2)[0]=p1.y;
        kf->statePost.ptr<float>(3)[0]=p2.y;
      }

      vector<Point2f>getStateL()
      {
        vector<Point2f>ret;
        //ret.push_back(Point2f(kf->statePost.ptr<float>(0)[0],-20));
        //ret.push_back(Point2f(kf->statePost.ptr<float>(1)[0],30));
        ret.push_back(Point2f(-5,kf->statePost.ptr<float>(0)[0]));
        ret.push_back(Point2f(15,kf->statePost.ptr<float>(1)[0]));

        return ret;
      }

      vector<Point2f>getStateR()
      {
        vector<Point2f>ret;


       // ret.push_back(Point2f(kf->statePost.ptr<float>(2)[0],-20));
       // ret.push_back(Point2f(kf->statePost.ptr<float>(3)[0],30));
        ret.push_back(Point2f(-5,kf->statePost.ptr<float>(2)[0]));
        ret.push_back(Point2f(15,kf->statePost.ptr<float>(3)[0]));
        return ret;
      }

      vector<Point2f>getPredictL()
      {
        vector<Point2f>ret;
        //ret.push_back(Point2f(prediction.ptr<float>(0)[0],-20));
        //ret.push_back(Point2f(prediction.ptr<float>(1)[0],30));
        ret.push_back(Point2f(-5,prediction.ptr<float>(0)[0]));
        ret.push_back(Point2f(15,prediction.ptr<float>(1)[0]));
        return ret;
      }


      vector<Point2f>getPredictR()
      {
        vector<Point2f>ret;
        //ret.push_back(Point2f(prediction.ptr<float>(2)[0],-20));
        //ret.push_back(Point2f(prediction.ptr<float>(3)[0],30));
        ret.push_back(Point2f(-5,prediction.ptr<float>(2)[0]));
        ret.push_back(Point2f(15,prediction.ptr<float>(3)[0]));
        return ret;
      }
      void next()
      {
        meas=Mat::zeros(4,1,CV_32FC1);
      }

      void addLine(Point p1,Point p2,bool left_flag)
      {
        Point2f l;
        int yc;
        ///交换点位置，确保第一个点位于上部，这样可以在变换到极坐标时保证0<beta<PI
        if(p1.x>p2.x)
          //if(p1.y>p2.y)
        {
          Point2f t;
          t=p1;
          p1=p2;
          p2=t;
        }

        yc=(p1.y+p2.y)/2;
          if(left_flag)
          {
            float previous_y1=kf->statePost.ptr<float>(0)[0];
            float previous_y2=kf->statePost.ptr<float>(1)[0];
            float center_intercept=(previous_y1+previous_y2)/2;
            if(p1.y==0&&p2.y==0)
            {
              meas.ptr<float>(0)[0]=kf->statePost.ptr<float>(0)[0];
              meas.ptr<float>(1)[0]=kf->statePost.ptr<float>(1)[0];
              return;
            }
            //if(abs(kf->statePost.ptr<float>(1)[0]-p2.y)>15)
            if(std::abs(center_intercept-yc)>1)
            {
              ///输入点车道线偏离过大，不取该车道线
              //meas.ptr<float>(0)[0]=state.ptr<float>(0)[0];
              //meas.ptr<float>(0)[0]=state.ptr<float>(1)[0];
                  left_wave_num=left_wave_num+1;
                  if(left_wave_num<=3)
                  {
                    meas.ptr<float>(0)[0]=kf->statePost.ptr<float>(0)[0];
                    meas.ptr<float>(1)[0]=kf->statePost.ptr<float>(1)[0];
                    return;
                  }
                  else
                  {
                    meas.ptr<float>(0)[0]=p1.y;
                    meas.ptr<float>(1)[0]=p2.y;
                    return;
                  }
            }
            else
            {
                meas.ptr<float>(0)[0]=p1.y;
                meas.ptr<float>(1)[0]=p2.y;
                return;
             }
            }
          else
          {
            float previous_y1=kf->statePost.ptr<float>(2)[0];
            float previous_y2=kf->statePost.ptr<float>(3)[0];
            float center_intercept=(previous_y1+previous_y2)/2;
            if(p1.y==0&&p2.y==0)
            {
              meas.ptr<float>(2)[0]=kf->statePost.ptr<float>(2)[0];
              meas.ptr<float>(3)[0]=kf->statePost.ptr<float>(3)[0];
              return;
            }
            ///右车道线
            //if(abs(xc-kf->statePost.ptr<float>(3)[0])>1)
           // else if(abs(kf->statePost.ptr<float>(3)[0]-p2.y)>15)
            else if(std::abs(center_intercept-yc)>1)
            {
              ///输入点车道线偏离过大，不取该车道线
              right_wave_num=right_wave_num+1;
                if(right_wave_num<=3)
                {
                  meas.ptr<float>(2)[0]=kf->statePost.ptr<float>(2)[0];
                  meas.ptr<float>(3)[0]=kf->statePost.ptr<float>(3)[0];

                }
                else
                {
                  meas.ptr<float>(2)[0]=p1.y;
                  meas.ptr<float>(3)[0]=p2.y;
                }
                return;
            }
            else
            {
              meas.ptr<float>(2)[0]=p1.y;
              meas.ptr<float>(3)[0]=p2.y;
              return;
            }
          }
      }
      void predict()
      {
        prediction=kf->predict();
        kf->correct(meas);

        if (left_wave_num >3 )
        {
          kf->statePost.ptr<float>(0)[0] = meas.ptr<float>(0)[0];
          kf->statePost.ptr<float>(1)[0] = meas.ptr<float>(1)[0];
          left_wave_num=0;
        }
        if(right_wave_num>3)
        {
          kf->statePost.ptr<float>(2)[0] = meas.ptr<float>(2)[0];
          kf->statePost.ptr<float>(3)[0] = meas.ptr<float>(3)[0];
          right_wave_num=0;
        }
        if(left_wave_num<=3&&left_wave_num>0)
        {
          kf->statePost.ptr<float>(0)[0] = meas.ptr<float>(0)[0];
          kf->statePost.ptr<float>(1)[0] = meas.ptr<float>(1)[0];
        }
        if(right_wave_num<=3&&right_wave_num>0)
        {
           kf->statePost.ptr<float>(2)[0] = meas.ptr<float>(2)[0];
           kf->statePost.ptr<float>(3)[0] = meas.ptr<float>(3)[0];
        }

        state=kf->statePost.t();

        std::cout<<"measure data:"<<meas.t()<<"\t"<<"predict data:"<<prediction.t()<<"\t"<<"update data:"<<kf->statePost.t()<<endl;
      }


};

#endif /* INCLUDE_CURB_DETECTION_LANEKALMANFILTER_H_ */

/*
if(meas.ptr<float>(0)[0]==0&&meas.ptr<float>(1)[0]==0&&meas.ptr<float>(1)[0]==0)
///没有车道线，直接将当前车道线作为车道线
{
   meas.ptr<float>(0)[0]=p1.x;
   meas.ptr<float>(1)[0]=p2.x;
}
else if(2*xc>meas.ptr<float>(0)[0]+state.ptr<float>(1)[0])
{
  meas.ptr<float>(0)[0]=p1.x;
  meas.ptr<float>(1)[0]=p2.x;
}
*/



/*

#include<opencv2/opencv.hpp>
#include<stdio.h>
#include<stdint.h>

using namespace std;
using namespace cv;
class LaneKalmanFilter
{
  public:
    LaneKalmanFilter(int stateNum,int measureNum)
    {
        stateNum_=stateNum;
        measureNum=stateNum;
        kf=new KalmanFilter(stateNum,measureNum,0);
        state=Mat(stateNum,1,CV_32FC1);
        meas=Mat(measureNum,1,CV_32FC1);
        prediction=Mat(stateNum,1,CV_32FC1);
        kf->processNoiseCov=Mat(4,1,CV_32FC1);
        kf->measurementNoiseCov=Mat(4,1,CV_32FC1);
        kf->transitionMatrix=Mat(4,4,CV_32FC1);
        kf->measurementMatrix=Mat(4,4,CV_32FC1);

        left_wave_num=0;
        right_wave_num=0;
      }

    void init()
    {
      m_bprior = true;

      meas = cv::Mat::zeros(4, 1, CV_32FC1);

      kf->init(4, 4, 0);

      setIdentity(kf->transitionMatrix, Scalar::all(1));
      setIdentity(kf->measurementMatrix, Scalar::all(1));

      setIdentity(kf->processNoiseCov, cvRealScalar(1e-7));
      setIdentity(kf->measurementNoiseCov, cvRealScalar(1e-5));
      setIdentity(kf->errorCovPost, Scalar::all(0.2));
      return ;
    }

      void setStateLaneL(Point2f p1, Point2f p2)
      {

        state.ptr<float>(0)[0] = p1.y;
        state.ptr<float>(1)[0] = p2.y;

        kf->statePost.ptr<float>(0)[0] = p1.y;
        kf->statePost.ptr<float>(1)[0] = p2.y;

      }
      void setStateLaneR(Point2f p1, Point2f p2)
      {

        state.ptr<float>(2)[0] = p1.y;
        state.ptr<float>(3)[0] = p2.y;
        kf->statePost.ptr<float>(2)[0] = p1.y;
        kf->statePost.ptr<float>(3)[0] = p2.y;
      }

      vector<Point2f> getStateL()
      {
        vector<Point2f> ret;
        ret.push_back(Point2f(-5, kf->statePost.ptr<float>(0)[0]));
        ret.push_back(Point2f(15, kf->statePost.ptr<float>(1)[0]));

        return ret;
      }

      vector<Point2f> getStateR()
      {
        vector<Point2f> ret;
        ret.push_back(Point2f(-5, kf->statePost.ptr<float>(2)[0]));
        ret.push_back(Point2f(15, kf->statePost.ptr<float>(3)[0]));
        return ret;
      }

      vector<Point2f> getPredictL()
      {
        vector<Point2f> ret;
        ret.push_back(Point2f(-5, prediction.ptr<float>(0)[0]));
        ret.push_back(Point2f(15, prediction.ptr<float>(1)[0]));
        return ret;
      }
      vector<Point2f> getPredictR()
      {
        vector<Point2f> ret;
        ret.push_back(Point2f(-5, prediction.ptr<float>(2)[0]));
        ret.push_back(Point2f(15, prediction.ptr<float>(3)[0]));
        return ret;
      }
      void next()
      {
        meas = Mat::zeros(4, 1, CV_32FC1);
      }

        void addLine(Point p1, Point p2, bool left_flag)
        {
          Point2f l;
          int yc;
          ///交换点位置，确保第一个点位于上部，这样可以在变换到极坐标时保证0<beta<PI
          if (p1.x > p2.x)
          //if(p1.y>p2.y)
          {
            Point2f t;
            t = p1;
            p1 = p2;
            p2 = t;
          }

          yc = (p1.y + p2.y) / 2;
          if (left_flag)
          {
            float previous_y1 = kf->statePost.ptr<float>(0)[0];
            float previous_y2 = kf->statePost.ptr<float>(1)[0];
            float center_intercept = (previous_y1 + previous_y2) / 2;
            if (p1.y == 0 && p2.y == 0)
            {
              meas.ptr<float>(0)[0] = kf->statePost.ptr<float>(0)[0];
              meas.ptr<float>(1)[0] = kf->statePost.ptr<float>(1)[0];
              return;
            }
            //if(abs(kf->statePost.ptr<float>(1)[0]-p2.y)>15)
            if (std::abs(center_intercept - yc) > 1)
            {
              ///输入点车道线偏离过大，不取该车道线
              left_wave_num = left_wave_num + 1;
              if (left_wave_num <= 4)
              {
                meas.ptr<float>(0)[0] = kf->statePost.ptr<float>(0)[0];
                meas.ptr<float>(1)[0] = kf->statePost.ptr<float>(1)[0];
                return;
              }
              else
              {
                meas.ptr<float>(0)[0] = p1.y;
                meas.ptr<float>(1)[0] = p2.y;
                return;
              }
            }
            else
            {
              meas.ptr<float>(0)[0] = p1.y;
              meas.ptr<float>(1)[0] = p2.y;
              return;
            }
          }
          else
          {
            float previous_y1 = kf->statePost.ptr<float>(2)[0];
            float previous_y2 = kf->statePost.ptr<float>(3)[0];
            float center_intercept = (previous_y1 + previous_y2) / 2;
            if (p1.y == 0 && p2.y == 0)
            {
              meas.ptr<float>(2)[0] = kf->statePost.ptr<float>(2)[0];
              meas.ptr<float>(3)[0] = kf->statePost.ptr<float>(3)[0];
              return;
            }
            ///右车道线
            //if(abs(xc-kf->statePost.ptr<float>(3)[0])>1)
            // else if(abs(kf->statePost.ptr<float>(3)[0]-p2.y)>15)
            else if (std::abs(center_intercept - yc) > 1)
            {
              ///输入点车道线偏离过大，不取该车道线
              right_wave_num = right_wave_num + 1;
              if (right_wave_num <= 4)
              {
                meas.ptr<float>(2)[0] = kf->statePost.ptr<float>(2)[0];
                meas.ptr<float>(3)[0] = kf->statePost.ptr<float>(3)[0];

              }
              else
              {
                meas.ptr<float>(2)[0] = p1.y;
                meas.ptr<float>(3)[0] = p2.y;
              }
              return;
            }
            else
            {
              meas.ptr<float>(2)[0] = p1.y;
              meas.ptr<float>(3)[0] = p2.y;
              return;
            }
          }
        }

      void predict()
      {
        prediction = kf->predict();
        kf->correct(meas);

        if (left_wave_num > 4)
        {
          kf->statePost.ptr<float>(0)[0] = meas.ptr<float>(0)[0];
          kf->statePost.ptr<float>(1)[0] = meas.ptr<float>(1)[0];
          left_wave_num = 0;
        }
        if (right_wave_num > 4)
        {
          kf->statePost.ptr<float>(2)[0] = meas.ptr<float>(2)[0];
          kf->statePost.ptr<float>(3)[0] = meas.ptr<float>(3)[0];
          right_wave_num = 0;
        }
        state = kf->statePost.t();
        std::cout << "measure data:" << meas.t() << "\t" << "predict data:" << prediction.t() << "\t" << "update data:" << kf->statePost.t() << endl;
      }

     void update(bool left_flag)
     {
       if(left_flag)
       {
             if(left_non_exit)
             {
               left_non_exit=false;
               prediction=kf->predict();
               kf->statePost.ptr<float>(0)[0]=prediction.ptr<float>(0)[0];
               kf->statePost.ptr<float>(1)[0]=prediction.ptr<float>(1)[0];
             }
             else if(!left_non_exit&&left_wave_num>=3)
             {
               left_wave_num=0;
               kf->statePost.ptr<float>(0)[0]=meas.ptr<float>(0)[0];
               kf->statePost.ptr<float>(1)[0]=meas.ptr<float>(1)[0];
             }
             else if(!left_non_exit&&left_wave_num>0&&left_wave_num<3)
             {
               prediction=kf->predict();
               kf->statePost.ptr<float>(0)[0]=prediction.ptr<float>(0)[0];
               kf->statePost.ptr<float>(1)[0]=prediction.ptr<float>(1)[0];
             }
             else if(!left_non_exit&&left_wave_num==0)
             {
               prediction=kf->predict();
              // meas.ptr<float>(2)[0]=0;
              // meas.ptr<float>(3)[0]=0;
               kf->correct(meas);
             }
       }
       else
       {
            if (right_non_exit)
            {
              right_non_exit = false;
              prediction = kf->predict();
              kf->statePost.ptr<float>(2)[0] = prediction.ptr<float>(2)[0];
              kf->statePost.ptr<float>(3)[0] = prediction.ptr<float>(3)[0];
            }
            else if (!right_non_exit && right_wave_num >= 3)
            {
              right_wave_num = 0;
              kf->statePost.ptr<float>(2)[0] = meas.ptr<float>(2)[0];
              kf->statePost.ptr<float>(3)[0] = meas.ptr<float>(3)[0];
            }
            else if (!right_non_exit && right_wave_num > 0 && right_wave_num < 3)
            {
              prediction = kf->predict();
              kf->statePost.ptr<float>(2)[0] = prediction.ptr<float>(2)[0];
              kf->statePost.ptr<float>(3)[0] = prediction.ptr<float>(3)[0];
            }
            else if (!right_non_exit && right_wave_num == 0)
            {
              prediction = kf->predict();
            //  meas.ptr<float>(0)[0]=0;
             // meas.ptr<float>(1)[0]=0;
              kf->correct(meas);
            }
    }
}
   void setMeas(Point p1, Point p2, bool left_flag)
    {
      Point2f l;
      int yc;
      ///交换点位置，确保第一个点位于上部，这样可以在变换到极坐标时保证0<beta<PI
      if (p1.x > p2.x)
      //if(p1.y>p2.y)
      {
        Point2f t;
        t = p1;
        p1 = p2;
        p2 = t;
      }

      yc = (p1.y + p2.y) / 2;
      if (left_flag)
      {
        float previous_y1 = kf->statePost.ptr<float>(0)[0];
        float previous_y2 = kf->statePost.ptr<float>(1)[0];
        float center_intercept = (previous_y1 + previous_y2) / 2;
        if (p1.y == 0 && p2.y == 0)
        {
          meas.ptr<float>(0)[0]= kf->statePost.ptr<float>(0)[0];
          meas.ptr<float>(1)[0]= kf->statePost.ptr<float>(1)[0];
          left_non_exit=true;
          return;
        }
        //if(abs(kf->statePost.ptr<float>(1)[0]-p2.y)>15)
        else if (std::abs(center_intercept - yc) > 1)
        {
          ///输入点车道线偏离过大，不取该车道线
          left_wave_num = left_wave_num + 1;
          meas.ptr<float>(0)[0] = p1.y;
          meas.ptr<float>(1)[0] = p2.y;
          return;
        }
        else
        {
          meas.ptr<float>(0)[0] = p1.y;
          meas.ptr<float>(1)[0] = p2.y;
          return;
        }
      }
      else
      {
        float previous_y1 = kf->statePost.ptr<float>(2)[0];
        float previous_y2 = kf->statePost.ptr<float>(3)[0];
        float center_intercept = (previous_y1 + previous_y2) / 2;
        if (p1.y == 0 && p2.y == 0)
        {
           meas.ptr<float>(2)[0]=kf->statePost.ptr<float>(2)[0];
           meas.ptr<float>(3)[0]=kf->statePost.ptr<float>(3)[0];
          right_non_exit=true;
          return;
        }
        ///右车道线
        //if(abs(xc-kf->statePost.ptr<float>(3)[0])>1)
        // else if(abs(kf->statePost.ptr<float>(3)[0]-p2.y)>15)
        else if (std::abs(center_intercept - yc) > 1)
        {
          ///输入点车道线偏离过大，不取该车道线
          right_wave_num = right_wave_num + 1;
          meas.ptr<float>(2)[0] = p1.y;
          meas.ptr<float>(3)[0] = p2.y;
          return;
        }
        else
        {
          meas.ptr<float>(2)[0] = p1.y;
          meas.ptr<float>(3)[0] = p2.y;
          return;
        }
      }
    }

public:
   int stateNum_;
   int measureNum_;
   Mat state,prediction,meas;
   KalmanFilter*kf;
   bool m_bprior;
   int left_wave_num;
   int right_wave_num;
   bool left_non_exit;
   bool right_non_exit;

};




#endif


*/

/*
 * LaneKalmanFilter.h
 *
 *  Created on: Aug 10, 2016
 *      Author: aicrobo
 */

