/***************************************************************************
 *   Copyright (C) 2005 by Nicola Bellotto                                 *
 *   nbellotto@lincoln.ac.uk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "bayes_tracking/trackwin.h"
#include <iostream>

const double TrackWin::NO_ORIENTATION=DBL_MAX;

TrackWin::TrackWin(const char* name, int width, int height, double scale)
{
    _name = new char[strlen(name)+1];
    strcpy(_name, name);
    _created = false;

    m_winWidth = width;
    m_winHeight = height;
    m_scale = scale;
    m_xoffset = m_winWidth / 2;
    m_yoffset = m_winHeight / 2;

    _x0 = 0;
    _y0 = 0;
    _th0 = 0;
    _cosTh0 = 1;
    _sinTh0 = 0;
    _text = new char[TEXT_LENGTH + 1];
    _text[TEXT_LENGTH] = _text[0] = '\0';
    _textColor = BLACK;
    _textPos = cvPoint((int)(20 * scale/SCALE_FACTOR), (int)(m_winHeight - 20 * scale/SCALE_FACTOR));
    cvInitFont(&_font, CV_FONT_HERSHEY_PLAIN, 0.5 * scale/SCALE_FACTOR, 0.5 * scale/SCALE_FACTOR, 0, 1, CV_AA);
}

void TrackWin::create()
{
    if (!_created)
    {
        // create image
        _canvas = cvCreateImage(cvSize(m_winWidth, m_winHeight), IPL_DEPTH_8U, 3);
        cvSet(_canvas, cvScalar(255, 255, 255));
        // create window
        cvNamedWindow(_name, CV_WINDOW_AUTOSIZE);
        cvShowImage(_name, _canvas);
        // process graphic events
        cvWaitKey(100);
    }
    else
        std::cerr << "TrackWin \"" << _name << "\" already created\n";
}

void TrackWin::destroy()
{
    if (_created)
    {
        cvDestroyWindow(_name);
        cvReleaseImage(&_canvas);
    }
}

TrackWin::~TrackWin()
{
    destroy();
    delete _text;
    delete _name;
}

void TrackWin::setObject(const char* label,
                         double xpos,
                         double ypos,
                         double orientation,
                         color_t color,
                         double varx,
                         double vary,
                         double covxy,
                         const ColMatrix* samples)
{
    t_object obj = {label, xpos, ypos, orientation, color, varx, vary, covxy, samples};
    _objVector.push_back(obj);
}


void TrackWin::setOrigin(double x0, double y0, double th0)
{
    _x0 = x0;
    _y0 = y0;
    _th0 = th0;
    _cosTh0 = cos(_th0);
    _sinTh0 = sin(_th0);
}


void TrackWin::setText(const char* text, color_t color)
{
    strncpy(_text, text, TEXT_LENGTH);
    _textColor = color;
}


const double maxRange = 10.;

void TrackWin::update(int delay, double cameraView, double cx, double cy)
{
    // clear the canvas
    cvSet(_canvas, cvScalar(235, 235, 235));
    // camera field of view
    if (cameraView > 0) {
        int npts = 3;
        CvPoint* pts = new CvPoint[3];
        int delta = (int)((maxRange * sin(cameraView/2.)) * m_scale);
        pts[0].x = m_xoffset - (int)(cy*m_scale);
        pts[0].y = m_yoffset - (int)(cx*m_scale);
        pts[1].x = delta + pts[0].x;
        pts[1].y = pts[0].y - (int)(maxRange * m_scale);
        pts[2].x = pts[0].x - delta;
        pts[2].y = pts[0].y - (int)(maxRange * m_scale);
        cvFillPoly(_canvas, &pts, &npts, 1, cvScalar(255, 255, 255));
        delete[] pts;
    }
    // draw the origin
    cvLine(_canvas,  // X axis
           cvPoint(m_xoffset, m_yoffset),
           cvPoint(m_scale + m_xoffset, m_yoffset),
           CV_RGB(192,192,192), 1);
    cvLine(_canvas,  // Y axis
           cvPoint(m_xoffset, m_yoffset),
           cvPoint(m_xoffset, -m_scale + m_yoffset),
           CV_RGB(192,192,192), 1);
    // draw all the objects
    for (uint i = 0; i < _objVector.size(); i++) {
        double dx = _objVector[i].x - _x0;
        double dy = _objVector[i].y - _y0;
        int x0 = (int)((dx * _cosTh0 + dy * _sinTh0) * m_scale + m_xoffset);
        int y0 = (int)(-(-dx * _sinTh0 + dy * _cosTh0) * m_scale + m_yoffset);
        CvScalar color = getColor(_objVector[i].color);
        // samples
        if (_objVector[i].s != NULL) {
            for (size_t n = 0; n < (_objVector[i].s)->size2(); n++) {
                double dx = (*_objVector[i].s)(0, n) - _x0;
                double dy = (*_objVector[i].s)(1, n) - _y0;
                int x = (int)((dx * _cosTh0 + dy * _sinTh0) * m_scale + m_xoffset);
                int y = (int)(-(-dx * _sinTh0 + dy * _cosTh0) * m_scale + m_yoffset);
                cvCircle(_canvas, cvPoint(x, y), 0, color, 1);
            }
        }
        // circle
        int r = (int)(0.25 * m_scale);
        cvCircle(_canvas, cvPoint(x0, y0), r, color, 1, CV_AA);
        // orientation
        if (_objVector[i].th != NO_ORIENTATION) {
            int x1 = x0 + (int)(r * cos(_objVector[i].th - _th0));
            int y1 = y0 - (int)(r * sin(_objVector[i].th - _th0));
            cvLine(_canvas, cvPoint(x0, y0), cvPoint(x1, y1), color, 1, CV_AA);
        }
        // variance
        if ((_objVector[i].varx > 0) && (_objVector[i].vary > 0)) {
	  //Covariance matrix of our data
	  double scale2 = m_scale*m_scale;
	  Mat covmat = (Mat_<double>(2,2) << _objVector[i].varx*scale2, _objVector[i].covxy*scale2, _objVector[i].covxy*scale2, _objVector[i].vary*scale2);	

	  //The mean of our data
	  Point2f mean(x0,y0);

	  //Calculate the error ellipse for a 95% confidence intervanl
	  RotatedRect elps = getErrorEllipse(2.4477, mean, covmat);
	  
	  Mat mc(_canvas);
	  ellipse(mc, elps, cv::Scalar(0,255,0), 1, CV_AA);
        }

        // label
        if (strlen(_objVector[i].label.c_str()) > 0)
            cvPutText(_canvas, _objVector[i].label.c_str(), cvPoint(x0, y0-r-5), &_font, color);
    }
    _objVector.clear();
    // draw the text
    if (strlen(_text) > 0)
        cvPutText(_canvas, _text, _textPos, &_font, getColor(_textColor));
    cvShowImage(_name, _canvas);
    // delete text buffer
    _text[0] = '\0';
    // process graphic events
    cvWaitKey(delay);
}




RotatedRect TrackWin::getErrorEllipse(double chisquare_val, Point2f mean, Mat covmat){

  //Get the eigenvalues and eigenvectors
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(covmat, true, eigenvalues, eigenvectors);

  //Calculate the angle between the largest eigenvector and the x-axis
  double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));

  //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
  if(angle < 0)
    angle += 6.28318530718;

  //Conver to degrees instead of radians
  angle = 180*angle/3.14159265359;

  //Calculate the size of the minor and major axes
  double halfmajoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(0));
  double halfminoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(1));

  //Return the oriented ellipse
  //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
  return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

}


CvScalar TrackWin::getColor(color_t color)
{
    switch (color)
    {
    case BLACK:
        return cvScalar(0, 0, 0);
        break;
    case WHITE:
        return cvScalar(255, 255, 255);
        break;
    case GREY:
        return cvScalar(128, 128, 128);
        break;
    case BLUE:
        return cvScalar(255, 0, 0);
        break;
    case GREEN:
        return cvScalar(0, 255, 0);
        break;
    case DARK_GREEN:
        return cvScalar(0, 127, 0);
        break;
    case RED:
        return cvScalar(0, 0, 255);
        break;
    case PURPLE:
        return cvScalar(196, 0, 196);
        break;
    case YELLOW:
        return cvScalar(0, 255, 255);
        break;
    case ORANGE:
        return cvScalar(0, 128, 255);
        break;
    case DARK_ORANGE:
        return cvScalar(0, 100, 200);
        break;
    default:
        std::cerr << "Wrong color code in:\n\tCvScalar TrackWin::getColor(color_t color)\n";
        return cvScalar(0, 0, 0);
    }
}


void TrackWin::saveSnapshot(const char* filename)
{
    cvSaveImage(filename, _canvas);
}
