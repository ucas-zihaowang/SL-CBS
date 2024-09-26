
#ifndef OPENCV_PLOT_H_
#define OPENCV_PLOT_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "common.h"
#include "copter_common.h"
#include "globalmap.h"


#define white   cv::Scalar(255, 255, 255)
#define black   cv::Scalar(0, 0, 0)
#define grey    cv::Scalar(100, 100, 100)
#define red     cv::Scalar(0, 0, 255)
#define blue    cv::Scalar(255, 0, 0)
#define green   cv::Scalar(0, 255, 0)
#define cyan    cv::Scalar(255, 255, 0)
#define magenta cv::Scalar(255, 0, 255)
#define yellow  cv::Scalar(0,255,255)



class OpenCVPlot
{
public:

    OpenCVPlot(GlobalMap2d &gmap): gmap(gmap)
    {
        const auto dim = gmap.getGridDim();

        img = cv::Mat( dim(1), dim(0), CV_8UC3, white );
    }


    void show( std::string window_name )
    {
        cv::namedWindow( window_name, CV_WINDOW_NORMAL );
        cv::imshow( window_name, img );
        cv::waitKey(0);
    }


    void save( std::string file_name )
    {
        cv::imwrite(file_name, img );
    }


    void drawPoints(const vec_Vec2f &pts, cv::Scalar color, int line_width = 1)
    {
        for ( const auto &it : pts )
        {
            const auto pn = gmap.toGrid(it);
            cv::Point pt(pn(0), pn(1));
            cv::rectangle( img, pt, pt, color, line_width );
        }
    }


    void drawCircle(const Vec2f &pt, cv::Scalar color, int r, int line_width = 1)
    {
        const auto pn = gmap.toGrid(pt);
        cv::circle( img, cv::Point(pn(0), pn(1)), r, color, line_width );
    }

    void drawSolidCircle(const Vec2f &pt, cv::Scalar color, int r)
    {
        const auto pn = gmap.toGrid(pt);
        cv::circle( img, cv::Point(pn(0), pn(1)), r, color, cv::FILLED );
    }


    void drawLineStrip(const vec_E<vec_Vec2f> &trias, cv::Scalar color, int line_width = 1) 
    {
        for (const auto &it: trias) 
        {
            for (size_t i = 0; i < it.size() - 1; i++) 
            {
                const auto pn1 = gmap.toGrid(it[i]);
                const auto pn2 = gmap.toGrid(it[i + 1]);
                cv::line( img, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)), color, line_width);
            }
        }
    }


    void drawTraj( const Trajectory2d &traj, cv::Scalar color, int line_width = 1, int num = 200 )
    {
        const auto ws = traj.sample(num);
        for (size_t i = 0; i < ws.size() - 1; i++)
        {
            const auto pn1 = gmap.toGrid(ws[i].pos);
            const auto pn2 = gmap.toGrid(ws[i + 1].pos);
            cv::line( img, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)), color, line_width );
        }
    }


    void drawDynamicTraj( std::string window_name, const Trajectory2d &traj, cv::Scalar color, int line_width = 1,  int num = 200 )
    {
        const auto ws = traj.sample(num);
        int ws_size = ws.size();
        bool wait = true;

        cv::namedWindow( window_name, CV_WINDOW_NORMAL );
        int index = 0;
        while ( 1 )
        {
            const auto pn1 = gmap.toGrid( ws[index].pos );
            const auto pn2 = gmap.toGrid( ws[index + 1].pos) ;
            cv::line( img, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)), color, line_width );
            cv::imshow( window_name, img );

            if ( wait )
            {
                cv::waitKey(0);
                wait = false;
            }

            int k = cv::waitKey(100) & 0xFF;
            if ( k == static_cast<int>('q') )
            {
                break;
            }

            index += 1;
            if ( index == (ws_size-1) )
            {
                index = 0;
                break;
            }
        }
        
        cv::imshow( window_name, img );
        cv::waitKey(0);
    }


    void drawDynamicProcess( std::string window_name, const Trajectory2d &traj, const std::vector<CopterState2d> &expanded_states, const std::vector<std::vector<Primitive2d>> &expanded_prs, const std::vector<std::vector<int>> &expanded_actions,cv::Scalar color, int line_width = 1)
    {
        int sample_num = 8;
        int expanded_num = expanded_states.size();

        std::vector<std::vector<vec_E<CopterState2d>>> wsss;


        for( int i = 0; i < expanded_num; i++ )
        {
            int iter_prs_num = expanded_prs[i].size();
            std::vector<vec_E<CopterState2d>> wss;
            for( int j = 0; j < iter_prs_num; j++ )
            {
                vec_E<CopterState2d> ws = expanded_prs[i][j].sample( sample_num );
                wss.push_back( ws );
            }
            wsss.push_back( wss );
        }

        cv::namedWindow( window_name, CV_WINDOW_NORMAL );
        cv::imshow( window_name, img );
        cv::waitKey(0);


        for( int i = 0; i  < expanded_num; i++ )
        {

            CopterState2d curr_expand_state = expanded_states[i];
            const auto pn = gmap.toGrid(curr_expand_state.pos);
            cv::circle( img, cv::Point(pn(0), pn(1)), 0.5, black, line_width );

            cv::imshow( window_name, img );


            int wss_size = wsss[i].size();
            for( int j = 0; j < wss_size; j++ )
            {
                vec_E<CopterState2d> ws = wsss[i][j];
                int ws_size = ws.size();
                cv::Scalar color_select = green;
                if ( expanded_actions[i][j] == -1 )
                    color_select = red;
                for( int k = 0; k < ws_size - 1; k++ )
                {
                    const auto pn1 = gmap.toGrid( ws[k].pos );
                    const auto pn2 = gmap.toGrid( ws[k + 1].pos );
                    cv::line( img, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)), color_select, line_width );
                }
            }
            cv::imshow( window_name, img );


            int k = cv::waitKey(100) & 0xFF;
            if ( k == static_cast<int>('q') )
            {
                break;
            }
        }

        const auto ws_f = traj.sample( sample_num * expanded_num );
        for (size_t i = 0; i < ws_f.size() - 1; i++)
        {
            const auto pn1 = gmap.toGrid(ws_f[i].pos);
            const auto pn2 = gmap.toGrid(ws_f[i + 1].pos);
            cv::line( img, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)), red, 2 );
        }

        cv::imshow( window_name, img );
        cv::waitKey(0);
    }


    void drawMultiDynamicTraj( std::string window_name, const std::vector<Trajectory2d> &trajs, cv::Scalar color, int line_width = 1 )
    {
        int sample_num = 8;
        int trajs_num = trajs.size();
        std::vector<vec_E<Command2d>> wss;
        int max_ws_size = 0;
        for( int i = 0; i < trajs_num; i++ )
        {
            int pr_num = trajs[i].getPrimitives().size();
            vec_E<Command2d> ws = trajs[i].sample( sample_num * pr_num );
            wss.push_back(ws);
            int tmp_ws_size = ws.size();
            if ( tmp_ws_size > max_ws_size )
                max_ws_size = tmp_ws_size;
        }
        bool wait = true;

        cv::namedWindow( window_name, CV_WINDOW_NORMAL );
        for( int i = 0; i < max_ws_size; i++ )
        {
            for( auto ws: wss )
            {
                int tmp_ws_size = ws.size();
                if ( i < (tmp_ws_size-1) )
                {
                    const auto pn1 = gmap.toGrid( ws[i].pos );
                    const auto pn2 = gmap.toGrid( ws[i + 1].pos );
                    cv::line( img, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)), color, line_width );
                }
            }
            cv::imshow( window_name, img );

            if ( wait )
            {
                cv::waitKey(0);
                wait = false;
            }

            int k = cv::waitKey(100) & 0xFF;
            if ( k == static_cast<int>('q') )
            {
                break;
            }

        }
        cv::imshow( window_name, img );
        cv::waitKey(0);
    }

private:
    GlobalMap2d &gmap;
    cv::Mat img; 
};

#endif
