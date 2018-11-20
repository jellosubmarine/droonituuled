#include "dt_config.hpp"
#include "offb_controller.hpp"
#include "offb_config.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cmath>

#ifdef OFFB_SHOW_VISUALS

#define OFFB_DISPLAY_NAME "Flight Controller Visuals"
#define OFFB_DISPLAY_WIDTH     500
#define OFFB_DISPLAY_HEIGHT    500
#define OFFB_DISPLAY_X_OFFSET  250
#define OFFB_DISPLAY_Y_OFFSET  250
#define OFFB_DISPLAY_M2PX      100

#define ODX(x) ((x) + OFFB_DISPLAY_X_OFFSET)
#define ODY(y) (OFFB_DISPLAY_HEIGHT - ((y) + OFFB_DISPLAY_Y_OFFSET))


void OffbController::drawGrid(double step) {
  double xm = 0.0, ym = 0.0;
  double xpx = ODX(xm), ypx = ODX(ym);

  while(xpx >= 0 && xpx < OFFB_DISPLAY_WIDTH) {
    cv::line( displayMatrix,
      cv::Point( xpx, 0 ),
      cv::Point( xpx, OFFB_DISPLAY_HEIGHT-1 ),
      cv::Scalar(100), 1
    );
    xm += step*OFFB_DISPLAY_M2PX;
    xpx = ODX(xm);
  }
  xm = 0.0;
  xpx = ODX(xm);

  while(xpx >= 0 && xpx < OFFB_DISPLAY_WIDTH) {
    cv::line( displayMatrix,
      cv::Point( xpx, 0 ),
      cv::Point( xpx, OFFB_DISPLAY_HEIGHT-1 ),
      cv::Scalar(100), 1
    );
    xm -= step*OFFB_DISPLAY_M2PX;
    xpx = ODX(xm);
  }

  while(ypx >= 0 && ypx < OFFB_DISPLAY_HEIGHT) {
    cv::line( displayMatrix,
      cv::Point( 0, ypx ),
      cv::Point( OFFB_DISPLAY_WIDTH-1, ypx ),
      cv::Scalar(100), 1
    );
    ym += step*OFFB_DISPLAY_M2PX;
    ypx = ODY(ym);
  }
  ym = 0.0;
  ypx = ODY(ym);

  while(ypx >= 0 && ypx < OFFB_DISPLAY_HEIGHT) {
    cv::line( displayMatrix,
      cv::Point( 0, ypx ),
      cv::Point( OFFB_DISPLAY_WIDTH-1, ypx ),
      cv::Scalar(100), 1
    );
    ym -= step*OFFB_DISPLAY_M2PX;
    ypx = ODY(ym);
  }
}

void OffbController::updateVisuals(double x, double y, double dir) {
  displayMatrix = cv::Scalar(0);

  drawGrid(1.0);
  cv::circle( displayMatrix,
    cv::Point(
      ODX(x * OFFB_DISPLAY_M2PX), ODY(y * OFFB_DISPLAY_M2PX)
    ), 8, cv::Scalar(160), -1
  );

  cv::line( displayMatrix,
    cv::Point( ODX(0), ODY(0) ),
    cv::Point( ODX(100 * sin(dir)), ODY(100 * cos(dir)) ),
    cv::Scalar(130), 3
  );

  cv::imshow(OFFB_DISPLAY_NAME, displayMatrix);
  cv::waitKey(1);
}

void OffbController::initVisuals() {
  displayMatrix = cv::Mat::zeros(
    OFFB_DISPLAY_HEIGHT, OFFB_DISPLAY_WIDTH, CV_8U
  );
  cv::namedWindow(OFFB_DISPLAY_NAME, cv::WINDOW_AUTOSIZE);
  //cv::resizeWindow(OFFB_DISPLAY_NAME, cv::Size(500,500));
  //cv::startWindowThread();
  cv::imshow(OFFB_DISPLAY_NAME, displayMatrix);
  cv::waitKey(1);
}

#endif
