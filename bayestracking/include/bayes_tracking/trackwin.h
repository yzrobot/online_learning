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
#ifndef TRACKWIN_H
#define TRACKWIN_H


#include "bayes_tracking/BayesFilter/bayesFlt.hpp"
#include "bayes_tracking/BayesFilter/matSup.hpp"
#include <opencv/highgui.h>
#include <vector>
#include <string>

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480
#define SCALE_FACTOR 30.0
#define TEXT_LENGTH 128

/**
@author Nicola Bellotto
*/

using namespace std;
using namespace Bayesian_filter_matrix;
using namespace cv;


class TrackWin {
public:
  typedef enum {BLACK, WHITE, GREY, BLUE, GREEN, DARK_GREEN, RED, PURPLE, YELLOW, ORANGE, DARK_ORANGE} color_t;

  typedef struct
  {
    std::string label;   // label
    double x;            // x pos
    double y;            // y pos
    double th;           // orientation
    color_t color;       // color
    double varx;          // x variance
    double vary;          // y variance
    double covxy;          // xy covariance
    const ColMatrix* s;  // samples
  } t_object;

  /**
  * Constructor
  * @param name Name of the window
  * @param width width of the window
  * @param height height of the window
  * @param scale scale of the window
  */
  TrackWin(const char* name,
          int width = WINDOW_WIDTH,
          int height = WINDOW_HEIGHT,
          double scale = SCALE_FACTOR);

  /**
  * Destructor
  */
  ~TrackWin();

  /**
  * Create and visualize the window
  */
  void create();

  /**
  * Destroy the window
  */
  void destroy();

  /**
  * Set an object to be drawn
  * @param label Label
  * @param xpos X position
  * @param ypos Y position
  * @param orientation Orientation
  * @param color Color
  * @param varx X variance
  * @param vary Y variance
  * @param covxy XY covariance
  * @param samples sample from particle filter
  */
  void setObject(const char* label, double xpos, double ypos, double orientation = NO_ORIENTATION,
                color_t color = BLACK, double varx = 0, double vary = 0, double covxy = 0, const ColMatrix* samples = NULL);

  /**
  * Set the origin (i.e. the observation point)
  * @param x0 X translation
  * @param y0 Y translation
  * @param th0 Rotation
  */
  void setOrigin(double x0, double y0, double th0);

  /**
  * Write some text on the window
  * @param text Text string
  * @param color Text color
  */
  void setText(const char* text, color_t color = BLACK);

  /**
  * Update the window's content
  * @param delay Time delay in [ms] for OpenCV' signal handling (default 5ms)
  * @param cameraView Angle of view of the camera
  * @param cx CX
  * @param cy CY
  */
  void update(int delay = 5, double cameraView = -1, double cx = 0, double cy = 0);

  /**
  * Save a snapshot of the current window
  * @param filename Name of the image file
  */
  void saveSnapshot(const char* filename);

public:
  static const double NO_ORIENTATION;
  
private:
  int m_winWidth;
  int m_winHeight;
  double m_scale;
  int m_xoffset;
  int m_yoffset;
  char* _name;
  bool _created;
  IplImage* _canvas;
  std::vector<t_object> _objVector;
  double _x0;
  double _y0;
  double _th0;
  double _cosTh0;
  double _sinTh0;
  char* _text;
  color_t _textColor;
  CvPoint _textPos;
  CvFont _font;

private:
  CvScalar getColor(color_t color);
  RotatedRect getErrorEllipse(double chisquare_val, Point2f mean, Mat covmat);
};

#endif
